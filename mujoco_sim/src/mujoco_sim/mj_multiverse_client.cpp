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

#include "mj_multiverse_client.h"

#include "mj_util.h"
#include <chrono>
#include <csignal>
#include <iostream>
#include <ros/ros.h>
#include <tinyxml2.h>
#include <zmq.hpp>

std::map<std::string, std::set<std::string>> MjMultiverseClient::send_objects;

std::map<std::string, std::set<std::string>> MjMultiverseClient::receive_objects;

static void validate_objects(tinyxml2::XMLDocument &doc, std::map<std::string, std::set<std::string>> &objects)
{
	do_each_child_element(doc.FirstChildElement(), "worldbody", [&](tinyxml2::XMLElement *worldbody_element)
						  { do_each_child_element(worldbody_element, [&](tinyxml2::XMLElement *body_element)
												  {
			int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, body_element->Attribute("name"));
			if (body_id == -1)
			{
				return;
			}

			std::set<std::string> &attributes = objects[body_element->Attribute("name")];
			for (const std::string &attribute : attributes)
			{
				if (strcmp(attribute.c_str(), "joint_rvalue") == 0 || strcmp(attribute.c_str(), "joint_tvalue") == 0)
				{
					do_each_child_element(body_element, "joint", [&](tinyxml2::XMLElement *joint_element)
					{
						const int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_element->Attribute("name"));
						if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE && strcmp(attribute.c_str(), "joint_rvalue") == 0)
						{
							objects[mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id)].insert("joint_rvalue");
						}
						else if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE && strcmp(attribute.c_str(), "joint_tvalue") == 0)
						{
							objects[mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id)].insert("joint_tvalue");
						}
					});
				}
			}

			attributes.erase("joint_rvalue");
			attributes.erase("joint_tvalue");

			if (attributes.size() == 0)
			{
				objects.erase(body_element->Attribute("name"));
			} }); });
}

void MjMultiverseClient::validate_objects()
{
	tinyxml2::XMLDocument doc;
	if (!load_XML(doc, model_path.c_str()))
	{
		ROS_WARN("Failed to load file \"%s\"\n", model_path.c_str());
		return;
	}

	::validate_objects(doc, send_objects);
	::validate_objects(doc, receive_objects);
}

MjMultiverseClient::MjMultiverseClient()
{
	host = "tcp://127.0.0.1";
	port = 7500;
}

MjMultiverseClient::~MjMultiverseClient()
{
}

void MjMultiverseClient::init(const std::string &in_host, const int in_port)
{
	host = in_host;
	port = in_port;

	init_objects();
}

void MjMultiverseClient::init_objects()
{
	XmlRpc::XmlRpcValue receive_object_params;
	if (ros::param::get("~receive", receive_object_params))
	{
		std::string log = "Set receive_objects: ";
		for (const std::pair<std::string, XmlRpc::XmlRpcValue> &receive_object_param : receive_object_params)
		{
			log += receive_object_param.first + " ";
			receive_objects[receive_object_param.first] = {};
			std::vector<std::string> receive_attributes;
			ros::param::get("~receive/" + receive_object_param.first, receive_attributes);
			for (const std::string &attribute : receive_attributes)
			{
				receive_objects[receive_object_param.first].insert(attribute);
			}
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
				if (strcmp(send_object_param.first.c_str(), "body") == 0)
				{
					for (const std::string &attribute : send_data)
					{
						if (strcmp(attribute.c_str(), "position") == 0 ||
							strcmp(attribute.c_str(), "quaternion") == 0 ||
							strcmp(attribute.c_str(), "relative_velocity") == 0 ||
							strcmp(attribute.c_str(), "force") == 0 ||
							strcmp(attribute.c_str(), "torque") == 0)
						{
							for (int body_id = 1; body_id < m->nbody; body_id++)
							{
								send_objects[mj_id2name(m, mjtObj::mjOBJ_BODY, body_id)].insert(attribute);
							}
						}
					}
				}
				else if (strcmp(send_object_param.first.c_str(), "joint") == 0)
				{
					for (const std::string &attribute : send_data)
					{
						if (strcmp(attribute.c_str(), "joint_rvalue") == 0)
						{
							for (int joint_id = 0; joint_id < m->njnt; joint_id++)
							{
								if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
								{
									send_objects[mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id)].insert(attribute);
								}
							}
						}
						else if (strcmp(attribute.c_str(), "joint_tvalue") == 0)
						{
							for (int joint_id = 0; joint_id < m->njnt; joint_id++)
							{
								if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
								{
									send_objects[mj_id2name(m, mjtObj::mjOBJ_JOINT, joint_id)].insert(attribute);
								}
							}
						}
					}
				}
				else
				{
					const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object_param.first.c_str());
					const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object_param.first.c_str());
					if (body_id != -1 || joint_id != -1)
					{
						log += send_object_param.first + " ";
						for (const std::string &attribute : send_data)
						{
							send_objects[send_object_param.first].insert(attribute);
						}
					}
				}
			}
		}
		ROS_INFO("%s", log.c_str());
	}

	validate_objects();
}

void MjMultiverseClient::connect()
{
	if (send_objects.size() > 0 || receive_objects.size() > 0)
	{
		context = zmq_ctx_new();

		socket_client = zmq_socket(context, ZMQ_REQ);
		socket_addr = host + ":" + std::to_string(port);

		ROS_INFO("Open the socket connection on %s", socket_addr.c_str());
		zmq_disconnect(socket_client, socket_addr.c_str());
		zmq_connect(socket_client, socket_addr.c_str());
		clear_data_vec();
		construct_meta_data();
		start_meta_data_thread();
	}
}

void MjMultiverseClient::start_meta_data_thread()
{
	meta_data_thread = std::thread(&MjMultiverseClient::send_and_receive_meta_data, this);
}

void MjMultiverseClient::clear_data_vec()
{
	send_data_vec.clear();
	receive_data_vec.clear();
}

void MjMultiverseClient::construct_meta_data()
{
	// Create JSON object and populate it
	Json::Value meta_data_json;
	meta_data_json["simulator"] = "mujoco";
	meta_data_json["length_unit"] = "m";
	meta_data_json["angle_unit"] = "rad";
	meta_data_json["force_unit"] = "N";
	meta_data_json["time_unit"] = "s";
	meta_data_json["handedness"] = "rhs";

	mtx.lock();
	for (const std::pair<std::string, std::set<std::string>> &send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
		if (body_id != -1)
		{
			const std::string body_name = send_object.first;
			const int body_ref_id = mj_name2id(m, mjtObj::mjOBJ_BODY, (body_name + "_ref").c_str());
			const int dof_id = m->body_dofadr[body_id];
			for (const std::string &attribute : send_object.second)
			{
				if (strcmp(attribute.c_str(), "position") == 0)
				{
					send_data_vec.emplace_back(&d->xpos[3 * body_id]);
					send_data_vec.emplace_back(&d->xpos[3 * body_id + 1]);
					send_data_vec.emplace_back(&d->xpos[3 * body_id + 2]);
				}
				else if (strcmp(attribute.c_str(), "quaternion") == 0)
				{
					send_data_vec.emplace_back(&d->xquat[4 * body_id]);
					send_data_vec.emplace_back(&d->xquat[4 * body_id + 1]);
					send_data_vec.emplace_back(&d->xquat[4 * body_id + 2]);
					send_data_vec.emplace_back(&d->xquat[4 * body_id + 3]);
				}
				else if (strcmp(attribute.c_str(), "force") == 0)
				{
					if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						if (contact_efforts.count(body_id) == 0)
						{
							contact_efforts[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
						}

						send_data_vec.emplace_back(&contact_efforts[body_id][0]);
						send_data_vec.emplace_back(&contact_efforts[body_id][1]);
						send_data_vec.emplace_back(&contact_efforts[body_id][2]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), body_name.c_str());
					}
				}
				else if (strcmp(attribute.c_str(), "torque") == 0)
				{
					if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						if (contact_efforts.count(body_id) == 0)
						{
							contact_efforts[body_id] = (mjtNum *)calloc(6, sizeof(mjtNum));
						}

						send_data_vec.emplace_back(&contact_efforts[body_id][3]);
						send_data_vec.emplace_back(&contact_efforts[body_id][4]);
						send_data_vec.emplace_back(&contact_efforts[body_id][5]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), body_name.c_str());
					}
				}
				else if (strcmp(attribute.c_str(), "relative_velocity") == 0)
				{
					if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						send_data_vec.emplace_back(&d->qvel[dof_id]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 1]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 2]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 3]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 4]);
						send_data_vec.emplace_back(&d->qvel[dof_id + 5]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), body_name.c_str());
					}
				}
				meta_data_json["send"][body_name].append(attribute);
			}
		}
		else if (joint_id != -1)
		{
			const std::string joint_name = send_object.first;
			const int qpos_id = m->jnt_qposadr[joint_id];
			for (const std::string &attribute : send_object.second)
			{
				if (strcmp(attribute.c_str(), "joint_rvalue") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
					{
						send_data_vec.emplace_back(&d->qpos[qpos_id]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), joint_name.c_str());
					}
				}
				else if (strcmp(attribute.c_str(), "joint_tvalue") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
					{
						send_data_vec.emplace_back(&d->qpos[qpos_id]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), joint_name.c_str());
					}
				}
				else if (strcmp(attribute.c_str(), "joint_position") == 0)
				{
					ROS_WARN("%s for %s not supported", attribute.c_str(), joint_name.c_str());
				}
				else if (strcmp(attribute.c_str(), "joint_quaternion") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL)
					{
						send_data_vec.emplace_back(&d->qpos[qpos_id]);
						send_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
						send_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
						send_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), joint_name.c_str());
					}
				}
				meta_data_json["send"][joint_name].append(attribute);
			}
		}
	}
	mtx.unlock();
	send_buffer_size = 1 + send_data_vec.size();

	mtx.lock();
	for (const std::pair<std::string, std::set<std::string>> &receive_object : receive_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, receive_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, receive_object.first.c_str());
		if (body_id != -1)
		{
			const std::string body_name = receive_object.first;
			const int body_ref_id = mj_name2id(m, mjtObj::mjOBJ_BODY, (body_name + "_ref").c_str());
			const int mocap_id = m->body_mocapid[body_ref_id];
			const int dof_id = m->body_dofadr[body_id];
			for (const std::string &attribute : receive_object.second)
			{
				if (strcmp(attribute.c_str(), "position") == 0)
				{
					if (body_ref_id == -1)
					{
						if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
						{
							int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							receive_data_vec.emplace_back(&d->qpos[qpos_id]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
						}
						else
						{
							ROS_WARN("%s for %s not supported", attribute.c_str(), body_name.c_str());
						}
					}
					else
					{
						receive_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id]);
						receive_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id + 1]);
						receive_data_vec.emplace_back(&d->mocap_pos[3 * mocap_id + 2]);
					}
				}
				else if (strcmp(attribute.c_str(), "quaternion") == 0)
				{
					if (body_ref_id == -1)
					{
						if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
						{
							int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 4]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 5]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 6]);
						}
						else if (m->body_dofnum[body_id] == 3 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_BALL)
						{
							int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							receive_data_vec.emplace_back(&d->qpos[qpos_id]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
							receive_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
						}
						else
						{
							ROS_WARN("%s for %s not supported", attribute.c_str(), body_name.c_str());
						}
					}
					else
					{
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id]);
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 1]);
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 2]);
						receive_data_vec.emplace_back(&d->mocap_quat[4 * mocap_id + 3]);
					}
				}
				else if (strcmp(attribute.c_str(), "force") == 0)
				{
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id]);
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id + 1]);
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id + 2]);
				}
				else if (strcmp(attribute.c_str(), "torque") == 0)
				{
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id + 3]);
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id + 4]);
					receive_data_vec.emplace_back(&d->xfrc_applied[6 * body_id + 5]);
				}
				else if (strcmp(attribute.c_str(), "relative_velocity") == 0)
				{
					if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
					{
						receive_data_vec.emplace_back(&d->qvel[dof_id]);
						receive_data_vec.emplace_back(&d->qvel[dof_id + 1]);
						receive_data_vec.emplace_back(&d->qvel[dof_id + 2]);
						receive_data_vec.emplace_back(&d->qvel[dof_id + 3]);
						receive_data_vec.emplace_back(&d->qvel[dof_id + 4]);
						receive_data_vec.emplace_back(&d->qvel[dof_id + 5]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), body_name.c_str());
					}
				}
				meta_data_json["receive"][body_name].append(attribute);
			}
		}
		else if (joint_id != -1)
		{
			const std::string joint_name = receive_object.first;
			const int qpos_id = m->jnt_qposadr[joint_id];
			for (const std::string &attribute : receive_object.second)
			{
				if (strcmp(attribute.c_str(), "joint_position") == 0)
				{
					ROS_WARN("%s for %s not supported", attribute.c_str(), joint_name.c_str());
				}
				else if (strcmp(attribute.c_str(), "joint_quaternion") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL)
					{
						receive_data_vec.emplace_back(&d->qpos[qpos_id]);
						receive_data_vec.emplace_back(&d->qpos[qpos_id + 1]);
						receive_data_vec.emplace_back(&d->qpos[qpos_id + 2]);
						receive_data_vec.emplace_back(&d->qpos[qpos_id + 3]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), joint_name.c_str());
					}
				}
				else if (strcmp(attribute.c_str(), "joint_rvalue") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE)
					{
						receive_data_vec.emplace_back(&d->qpos[qpos_id]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), joint_name.c_str());
					}
				}
				else if (strcmp(attribute.c_str(), "joint_tvalue") == 0)
				{
					if (m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE)
					{
						receive_data_vec.emplace_back(&d->qpos[qpos_id]);
					}
					else
					{
						ROS_WARN("%s for %s not supported", attribute.c_str(), joint_name.c_str());
					}
				}
				meta_data_json["receive"][joint_name].append(attribute);
			}
		}
	}
	mtx.unlock();
	receive_buffer_size = 1 + receive_data_vec.size();

	meta_data_str = meta_data_json.toStyledString();
}

void MjMultiverseClient::send_and_receive_meta_data()
{
	ROS_INFO("%s", meta_data_str.c_str());

	zmq_msg_t response_message;
	zmq_msg_init(&response_message);

	std::map<std::string, size_t> response_buffer_sizes = {{"send", 1}, {"receive", 1}};

	while (ros::ok())
	{
		// Send JSON string over ZMQ
		zmq_send(socket_client, meta_data_str.c_str(), meta_data_str.size(), 0);

		// Receive response over ZMQ
		zmq_msg_recv(&response_message, socket_client, 0);

		Json::Reader reader;
		std::string response_message_str(static_cast<char *>(zmq_msg_data(&response_message)), zmq_msg_size(&response_message));
		reader.parse(response_message_str, meta_data_res_json);

		if (meta_data_res_json["time"] < 0)
		{
			zmq_msg_init(&response_message);
			ROS_WARN("The socket server at %s has been terminated, resend the message", socket_addr.c_str());
			zmq_disconnect(socket_client, socket_addr.c_str());
			zmq_connect(socket_client, socket_addr.c_str());
		}
		else
		{
			for (std::pair<const std::string, size_t> &response_buffer_size : response_buffer_sizes)
			{
				const Json::Value json_object = meta_data_res_json[response_buffer_size.first];
				for (auto object_it = json_object.begin(); object_it != json_object.end(); ++object_it)
				{
					const std::string object_name = object_it.key().asString();
					const Json::Value json_object_data = json_object[object_name];
					for (auto object_data_it = json_object_data.begin(); object_data_it != json_object_data.end(); ++object_data_it)
					{
						const std::string attribute_name = object_data_it.key().asString();
						response_buffer_size.second += json_object_data[attribute_name].size();
					}
				}
			}

			break;
		}
	}

	if (response_buffer_sizes["send"] != send_buffer_size || response_buffer_sizes["receive"] != receive_buffer_size)
	{
		ROS_ERROR("Failed to initialize the socket at %s: send_buffer_size(server = %ld, client = %ld), receive_buffer_size(server = %ld, client = %ld).",
				  socket_addr.c_str(),
				  response_buffer_sizes["send"],
				  send_buffer_size,
				  response_buffer_sizes["receive"],
				  receive_buffer_size);
		zmq_disconnect(socket_client, socket_addr.c_str());
		return;
	}

	bind_object_data();

	ROS_INFO("Initialized the socket at %s successfully.", socket_addr.c_str());
	ROS_INFO("Start communication on %s (send: %ld, receive: %ld)", socket_addr.c_str(), send_buffer_size, receive_buffer_size);
	send_buffer = (double *)calloc(send_buffer_size, sizeof(double));
	receive_buffer = (double *)calloc(receive_buffer_size, sizeof(double));
	is_enabled = true;
}

void MjMultiverseClient::bind_object_data()
{
	mtx.lock();
	for (const std::pair<std::string, std::set<std::string>> &send_object : send_objects)
	{
		const int body_id = mj_name2id(m, mjtObj::mjOBJ_BODY, send_object.first.c_str());
		const int joint_id = mj_name2id(m, mjtObj::mjOBJ_JOINT, send_object.first.c_str());
		if (body_id != -1)
		{
			if (m->body_dofnum[body_id] == 6 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_FREE)
			{
				mjtNum *xpos_desired = d->xpos + 3 * body_id;
				mjtNum *xquat_desired = d->xquat + 4 * body_id;

				for (const std::string &attribute : send_object.second)
				{
					if (strcmp(attribute.c_str(), "position") == 0)
					{
						const double x = meta_data_res_json["send"][send_object.first][attribute][0].asDouble();
						const double y = meta_data_res_json["send"][send_object.first][attribute][1].asDouble();
						const double z = meta_data_res_json["send"][send_object.first][attribute][2].asDouble();
						if (std::isnan(x) && std::isnan(y) && std::isnan(z))
						{
							xpos_desired[0] = x;
							xpos_desired[1] = y;
							xpos_desired[2] = z;
						}
					}
					else if (strcmp(attribute.c_str(), "quaternion") == 0)
					{
						const double w = meta_data_res_json["send"][send_object.first][attribute][0].asDouble();
						const double x = meta_data_res_json["send"][send_object.first][attribute][1].asDouble();
						const double y = meta_data_res_json["send"][send_object.first][attribute][2].asDouble();
						const double z = meta_data_res_json["send"][send_object.first][attribute][3].asDouble();
						if (std::isnan(w) && std::isnan(x) && std::isnan(y) && std::isnan(z))
						{
							xquat_desired[0] = w;
							xquat_desired[1] = x;
							xquat_desired[2] = y;
							xquat_desired[3] = z;
						}
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
			else if (m->body_dofnum[body_id] == 3 && m->body_jntadr[body_id] != -1 && m->jnt_type[m->body_jntadr[body_id]] == mjtJoint::mjJNT_BALL)
			{
				for (const std::string &attribute : send_object.second)
				{
					if (strcmp(attribute.c_str(), "quaternion") == 0)
					{
						const double w = meta_data_res_json["send"][send_object.first][attribute][0].asDouble();
						const double x = meta_data_res_json["send"][send_object.first][attribute][1].asDouble();
						const double y = meta_data_res_json["send"][send_object.first][attribute][2].asDouble();
						const double z = meta_data_res_json["send"][send_object.first][attribute][3].asDouble();

						if (std::isnan(w) && std::isnan(x) && std::isnan(y) && std::isnan(z))
						{
							const mjtNum xquat_desired[4] = {w, x, y, z};
							mjtNum *xquat_current_neg = d->xquat + 4 * body_id;
							mju_negQuat(xquat_current_neg, xquat_current_neg);

							const int qpos_id = m->jnt_qposadr[m->body_jntadr[body_id]];
							mju_mulQuat(d->qpos + qpos_id, xquat_current_neg, xquat_desired);
						}
					}
				}
			}
		}
		else if (joint_id != -1)
		{
			for (const std::string &attribute : send_object.second)
			{
				if ((strcmp(attribute.c_str(), "joint_rvalue") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_HINGE) ||
					(strcmp(attribute.c_str(), "joint_tvalue") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_SLIDE))
				{
					const double v = meta_data_res_json["send"][send_object.first][attribute][0].asDouble();
					if (std::isnan(v))
					{
						const int qpos_id = m->jnt_qposadr[joint_id];
						d->qpos[qpos_id] = v;
					}
				}
				else if ((strcmp(attribute.c_str(), "joint_quaternion") == 0 && m->jnt_type[joint_id] == mjtJoint::mjJNT_BALL))
				{
					const double w = meta_data_res_json["send"][send_object.first][attribute][0].asDouble();
					const double x = meta_data_res_json["send"][send_object.first][attribute][1].asDouble();
					const double y = meta_data_res_json["send"][send_object.first][attribute][2].asDouble();
					const double z = meta_data_res_json["send"][send_object.first][attribute][3].asDouble();

					if (std::isnan(w) && std::isnan(x) && std::isnan(y) && std::isnan(z))
					{
						const int qpos_id = m->jnt_qposadr[joint_id];
						d->qpos[qpos_id] = w;
						d->qpos[qpos_id + 1] = x;
						d->qpos[qpos_id + 2] = y;
						d->qpos[qpos_id + 3] = z;
					}
				}
			}
		}
	}
	mtx.unlock();
}

double MjMultiverseClient::get_time_now()
{
	return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void MjMultiverseClient::bind_send_data()
{
	for (std::pair<const int, mjtNum *> &contact_effort : contact_efforts)
	{
		mjtNum jac[6 * m->nv];
		mj_jacBodyCom(m, d, jac, jac + 3 * m->nv, contact_effort.first);
		mju_mulMatVec(contact_effort.second, jac, d->qfrc_constraint, 6, m->nv);
	}

	*send_buffer = get_time_now();

	for (size_t i = 0; i < send_buffer_size - 1; i++)
	{
		send_buffer[i + 1] = *send_data_vec[i];
	}
}

void MjMultiverseClient::bind_receive_data()
{
	for (size_t i = 0; i < receive_buffer_size - 1; i++)
	{
		*receive_data_vec[i] = receive_buffer[i + 1];
	}
}

void MjMultiverseClient::communicate()
{
	if (is_enabled)
	{
		bind_send_data();

		zmq_send(socket_client, send_buffer, send_buffer_size * sizeof(double), 0);

		zmq_recv(socket_client, receive_buffer, receive_buffer_size * sizeof(double), 0);

		if (*receive_buffer < 0)
		{
			is_enabled = false;
			ROS_WARN("The socket server at %s has been terminated, resend the message", socket_addr.c_str());
			stop_meta_data_thread();
			start_meta_data_thread();
			return;
		}

		bind_receive_data();
	}
}

void MjMultiverseClient::clean_up()
{
	for (std::pair<const int, mjtNum *> &contact_effort : contact_efforts)
	{
		free(contact_effort.second);
	}
}

void MjMultiverseClient::stop_meta_data_thread()
{
	if (meta_data_thread.joinable())
	{
		meta_data_thread.join();
	}
}

void MjMultiverseClient::disconnect()
{
	std::cout << "Closing the socket client on " << socket_addr << std::endl;
	if (is_enabled)
	{
		const std::string close_data = "{}";

		zmq_send(socket_client, close_data.c_str(), close_data.size(), 0);

		free(send_buffer);
		free(receive_buffer);

		clean_up();

		zmq_disconnect(socket_client, socket_addr.c_str());

		is_enabled = false;
	}

	zmq_ctx_shutdown(context);

	if (meta_data_thread.joinable())
	{
		meta_data_thread.join();
	}
}