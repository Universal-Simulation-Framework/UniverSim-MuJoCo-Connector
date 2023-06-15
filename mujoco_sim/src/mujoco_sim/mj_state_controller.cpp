// Copyright (c) 2023, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "mj_state_controller.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <zmq.hpp>

std::string host = "tcp://127.0.0.1";

std::map<std::string, std::vector<std::string>> MjStateController::send_objects;

std::map<std::string, std::vector<std::string>> MjStateController::receive_objects;

bool should_shut_down = false;

MjStateController::~MjStateController()
{
}

void MjStateController::init(const int port)
{
	XmlRpc::XmlRpcValue receive_object_params;
	if (ros::param::get("~receive", receive_object_params))
	{
		std::string log = "Set receive_objects: ";
		for (const std::pair<std::string, XmlRpc::XmlRpcValue> &receive_object_param : receive_object_params)
		{
			log += receive_object_param.first + " ";
			receive_objects[receive_object_param.first] = {};
			ros::param::get("~receive/" + receive_object_param.first, receive_objects[receive_object_param.first]);
		}
		ROS_INFO("%s", log.c_str());
	}

	XmlRpc::XmlRpcValue send_object_params;
	if (ros::param::get("~send", send_object_params))
	{
		std::string log = "Set send_objects: ";
		for (const std::pair<std::string, XmlRpc::XmlRpcValue> &send_object_param : send_object_params)
		{
			std::vector<std::string> send_data;
			if (ros::param::get("~send/" + send_object_param.first, send_data))
			{
				const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object_param.first.c_str());
				const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object_param.first.c_str());
				if (body_id != -1 || joint_id != -1)
				{
					log += send_object_param.first + " ";
					send_objects[send_object_param.first] = send_data;
				}
			}
		}
		ROS_INFO("%s", log.c_str());
	}

	if (send_objects.size() > 0 || receive_objects.size() > 0)
	{
		context = zmq_ctx_new();

		socket_client = zmq_socket(context, ZMQ_REQ);
		socket_addr = host + ":" + std::to_string(port);

		ROS_INFO("Open the socket connection on %s", socket_addr.c_str());
		send_meta_data();
	}
}

void MjStateController::send_meta_data()
{
	zmq_disconnect(socket_client, socket_addr.c_str());
	zmq_connect(socket_client, socket_addr.c_str());

	send_data_vec.clear();
	receive_data_vec.clear();

	send_meta_data_thread = std::thread([this]()
										{
		// Create JSON object and populate it
		Json::Value meta_data_json;
		meta_data_json["time"] = "microseconds";
		meta_data_json["simulator"] = "mujoco";
		meta_data_json["length_unit"] = "m";
		meta_data_json["angle_unit"] = "rad";
		meta_data_json["force_unit"] = "N";
		meta_data_json["handedness"] = "rhs";

		mtx.lock();
		for (const std::pair<std::string, std::vector<std::string>> &send_object : send_objects)
		{
			const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
			const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
			for (const std::string &attribute : send_object.second)
			{
				if (strcmp(attribute.c_str(), "position") == 0)
				{
					send_data_vec.push_back(&d->xpos[3 * body_id]);
					send_data_vec.push_back(&d->xpos[3 * body_id + 1]);
					send_data_vec.push_back(&d->xpos[3 * body_id + 2]);
				}
				else if (strcmp(attribute.c_str(), "quaternion") == 0)
				{
					send_data_vec.push_back(&d->xquat[4 * body_id]);
					send_data_vec.push_back(&d->xquat[4 * body_id + 1]);
					send_data_vec.push_back(&d->xquat[4 * body_id + 2]);
					send_data_vec.push_back(&d->xquat[4 * body_id + 3]);
				}
				else if (strcmp(attribute.c_str(), "joint_rvalue") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
					{
						const int qpos_id = m->jnt_qposadr[joint_id];
						send_data_vec.push_back(&d->qpos[qpos_id]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), send_object.first.c_str());
					}
				}
				else if (strcmp(attribute.c_str(), "joint_tvalue") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
					{
						const int qpos_id = m->jnt_qposadr[joint_id];
						send_data_vec.push_back(&d->qpos[qpos_id]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), send_object.first.c_str());
					}
				}
				else if (strcmp(attribute.c_str(), "joint_position") == 0)
				{
					ROS_WARN("%s for %s not implemented yet", attribute.c_str(), send_object.first.c_str());
				}
				else if (strcmp(attribute.c_str(), "joint_quaternion") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL)
					{
						const int qpos_id = m->jnt_qposadr[joint_id];
						send_data_vec.push_back(&d->qpos[qpos_id]);
						send_data_vec.push_back(&d->qpos[qpos_id + 1]);
						send_data_vec.push_back(&d->qpos[qpos_id + 2]);
						send_data_vec.push_back(&d->qpos[qpos_id + 3]);
					}
					else
					{
						ROS_WARN("%s for %s not implemented yet", attribute.c_str(), send_object.first.c_str());
					}
				}
				else if (strcmp(attribute.c_str(), "force") == 0)
				{
					const int dof_id = m->body_dofadr[body_id];
					const int dof_num = m->body_dofnum[body_id];
					if (dof_num == 6)
					{
						send_data_vec.push_back(&d->qfrc_constraint[dof_id]);
						send_data_vec.push_back(&d->qfrc_constraint[dof_id + 1]);
						send_data_vec.push_back(&d->qfrc_constraint[dof_id + 2]);
					}
				}
				else if (strcmp(attribute.c_str(), "torque") == 0)
				{
					const int dof_id = m->body_dofadr[body_id];
					const int dof_num = m->body_dofnum[body_id];
					if (dof_num == 6)
					{
						send_data_vec.push_back(&d->qfrc_constraint[dof_id + 3]);
						send_data_vec.push_back(&d->qfrc_constraint[dof_id + 4]);
						send_data_vec.push_back(&d->qfrc_constraint[dof_id + 5]);
					}
				}
				meta_data_json["send"][send_object.first].append(attribute);
			}
		}
		mtx.unlock();
		send_buffer_size = 1 + send_data_vec.size();

		mtx.lock();
		for (const std::pair<std::string, std::vector<std::string>> &receive_object : receive_objects)
		{
			const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, (receive_object.first).c_str());
			const int body_ref_id = mj_name2id(m, mjtObj::mjOBJ_BODY, (receive_object.first + "_ref").c_str());
			const int mocap_id = m->body_mocapid[body_ref_id];
			for (const std::string &attribute : receive_object.second)
			{
				if (strcmp(attribute.c_str(), "position") == 0)
				{
					if (body_ref_id == -1)
					{
						if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
						{
							int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							receive_data_vec.push_back(&d->qpos[qpos_id]);
							receive_data_vec.push_back(&d->qpos[qpos_id + 1]);
							receive_data_vec.push_back(&d->qpos[qpos_id + 2]);
						}
						else
						{
							ROS_WARN("Not implemented yet");
						}
					}
					else
					{
						receive_data_vec.push_back(&d->mocap_pos[3 * body_id]);
						receive_data_vec.push_back(&d->mocap_pos[3 * body_id + 1]);
						receive_data_vec.push_back(&d->mocap_pos[3 * body_id + 2]);
					}
				}
				else if (strcmp(attribute.c_str(), "quaternion") == 0)
				{
					if (body_ref_id == -1)
					{
						if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
						{
							int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							receive_data_vec.push_back(&d->qpos[qpos_id + 3]);
							receive_data_vec.push_back(&d->qpos[qpos_id + 4]);
							receive_data_vec.push_back(&d->qpos[qpos_id + 5]);
							receive_data_vec.push_back(&d->qpos[qpos_id + 6]);
						}
						else
						{
							ROS_WARN("Not implemented yet");
						}
					}
					else
					{
						receive_data_vec.push_back(&d->mocap_quat[4 * mocap_id]);
						receive_data_vec.push_back(&d->mocap_quat[4 * mocap_id + 1]);
						receive_data_vec.push_back(&d->mocap_quat[4 * mocap_id + 2]);
						receive_data_vec.push_back(&d->mocap_quat[4 * mocap_id + 3]);
					}
				}
				else if (strcmp(attribute.c_str(), "force") == 0)
				{
					receive_data_vec.push_back(&d->xfrc_applied[6 * body_id]);
					receive_data_vec.push_back(&d->xfrc_applied[6 * body_id + 1]);
					receive_data_vec.push_back(&d->xfrc_applied[6 * body_id + 2]);
				}
				else if (strcmp(attribute.c_str(), "torque") == 0)
				{
					receive_data_vec.push_back(&d->xfrc_applied[6 * body_id + 3]);
					receive_data_vec.push_back(&d->xfrc_applied[6 * body_id + 4]);
					receive_data_vec.push_back(&d->xfrc_applied[6 * body_id + 5]);
				}
				meta_data_json["receive"][receive_object.first].append(attribute);
			}
		}
		mtx.unlock();
		receive_buffer_size = 1 + receive_data_vec.size();

		double *buffer = (double *)calloc(send_buffer_size + 2, sizeof(double));
		const std::string meta_data_str = meta_data_json.toStyledString();

		ROS_INFO("%s", meta_data_str.c_str());
		
		while (true)
		{
			// Send JSON string over ZMQ
			zmq_send(socket_client, meta_data_str.c_str(), meta_data_str.size(), 0);

			// Receive buffer sizes and send_data (if exists) over ZMQ
			zmq_recv(socket_client, buffer, (send_buffer_size + 2) * sizeof(double), 0);
			if (*buffer < 0)
			{
				free(buffer);
				buffer = (double *)calloc(send_buffer_size + 2, sizeof(double));
				ROS_WARN("The socket server at %s has been terminated, resend the message", socket_addr.c_str());
				zmq_disconnect(socket_client, socket_addr.c_str());
				zmq_connect(socket_client, socket_addr.c_str());
			}
			else
			{
				break;
			}
		}

		size_t recv_buffer_size[2] = {(size_t)buffer[0], (size_t)buffer[1]};
		if (recv_buffer_size[0] != send_buffer_size || recv_buffer_size[1] != receive_buffer_size)
		{
			ROS_ERROR("Failed to initialize the socket at %s: send_buffer_size(server = %ld, client = %ld), receive_buffer_size(server = %ld, client = %ld).", 
				socket_addr.c_str(), 
				recv_buffer_size[0], 
				send_buffer_size, 
				recv_buffer_size[1], 
				receive_buffer_size);
			zmq_disconnect(socket_client, socket_addr.c_str());
		}
		else
		{
			if (buffer[2] < 0.0)
			{
				ROS_INFO("Continue state on socket %s", socket_addr.c_str());
				mtx.lock();

				double *buffer_addr = buffer + 3;
				
				for (const std::pair<std::string, std::vector<std::string>> &send_object : send_objects)
				{
					const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
					const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
					if (body_id != -1 && m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						mjtNum xpos_desired[3] = {d->xpos[3 * body_id], d->xpos[3 * body_id + 1], d->xpos[3 * body_id + 2]};
						mjtNum xquat_desired[4] = {d->xquat[4 * body_id], d->xquat[4 * body_id + 1], d->xquat[4 * body_id + 2], d->xquat[4 * body_id + 3]};

						for (const std::string &attribute : send_object.second)
						{
							if (strcmp(attribute.c_str(), "position") == 0)
							{
								xpos_desired[0] = *buffer_addr++;
								xpos_desired[1] = *buffer_addr++;
								xpos_desired[2] = *buffer_addr++;
							}
							else if (strcmp(attribute.c_str(), "quaternion") == 0)
							{
								xquat_desired[0] = *buffer_addr++;
								xquat_desired[1] = *buffer_addr++;
								xquat_desired[2] = *buffer_addr++;
								xquat_desired[2] = *buffer_addr++;
							}
						}

						const int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
						d->qpos[qpos_id] = xpos_desired[0];
						d->qpos[qpos_id + 1] = xpos_desired[1];
						d->qpos[qpos_id + 2] = xpos_desired[2];
						d->qpos[qpos_id + 3] = xquat_desired[0];
						d->qpos[qpos_id + 4] = xquat_desired[1];
						d->qpos[qpos_id + 5] = xquat_desired[2];
						d->qpos[qpos_id + 6] = xquat_desired[3];
					}
					else if (body_id != -1 && m->body_dofnum[body_id] == 3 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_BALL)
					{
						for (const std::string &attribute : send_object.second)
						{
							if (strcmp(attribute.c_str(), "position") == 0)
							{
								buffer_addr += 3;
							}
							else if (strcmp(attribute.c_str(), "quaternion") == 0)
							{
								const mjtNum xquat_desired[4] = {*buffer_addr++, *buffer_addr++, *buffer_addr++, *buffer_addr++};
								mjtNum xquat_current_neg[4] = {d->xquat[4 * body_id], d->xquat[4 * body_id + 1], d->xquat[4 * body_id + 2], d->xquat[4 * body_id + 3]};
								mju_negQuat(xquat_current_neg, xquat_current_neg);
								mjtNum qpos[4] = {1.0, 0.0, 0.0, 0.0};
								mju_mulQuat(qpos, xquat_current_neg, xquat_desired);
								const int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
								d->qpos[qpos_id] = qpos[0];
								d->qpos[qpos_id + 1] = qpos[1];
								d->qpos[qpos_id + 2] = qpos[2];
								d->qpos[qpos_id + 3] = qpos[3];
							}
						}
					}
					else if (joint_id != -1)
					{
						if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE || m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
						{
							const int qpos_id = m->jnt_qposadr[joint_id];
							d->qpos[qpos_id] = *buffer_addr++;
						}
						else if (m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL)
						{
							const int qpos_id = m->jnt_qposadr[joint_id];
							d->qpos[qpos_id] = *buffer_addr++;
							d->qpos[qpos_id + 1] = *buffer_addr++;
							d->qpos[qpos_id + 2] = *buffer_addr++;
						}
					}
				}

				mtx.unlock();
			}

			ROS_INFO("Initialized the socket at %s successfully.", socket_addr.c_str());
			ROS_INFO("Start communication on %s (send: %ld, receive: %ld)", socket_addr.c_str(), send_buffer_size, receive_buffer_size);
			send_buffer = (double *)calloc(send_buffer_size, sizeof(double));
			receive_buffer = (double *)calloc(receive_buffer_size, sizeof(double));
			is_enabled = true;
		}

		free(buffer); });
}

void MjStateController::communicate()
{
	if (is_enabled)
	{
		*send_buffer = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

		for (size_t i = 0; i < send_buffer_size - 1; i++)
		{
			send_buffer[i + 1] = *send_data_vec[i];
		}

		zmq_send(socket_client, send_buffer, send_buffer_size * sizeof(double), 0);

		zmq_recv(socket_client, receive_buffer, receive_buffer_size * sizeof(double), 0);

		if (*receive_buffer < 0)
		{
			is_enabled = false;
			ROS_WARN("The socket server at %s has been terminated, resend the message", socket_addr.c_str());
			if (send_meta_data_thread.joinable())
			{
				send_meta_data_thread.join();
			}
			send_meta_data();
			return;
		}

		for (size_t i = 0; i < receive_buffer_size - 1; i++)
		{
			*receive_data_vec[i] = receive_buffer[i + 1];
		}
	}
}

void MjStateController::deinit()
{
	ROS_INFO("Closing the socket client on %s", socket_addr.c_str());
	if (is_enabled)
	{
		const std::string close_data = "{}";

		zmq_send(socket_client, close_data.c_str(), close_data.size(), 0);

		free(send_buffer);
		free(receive_buffer);

		zmq_disconnect(socket_client, socket_addr.c_str());
	}
	else if (send_meta_data_thread.joinable())
	{
		zmq_ctx_shutdown(context);
		send_meta_data_thread.join();
	}
}