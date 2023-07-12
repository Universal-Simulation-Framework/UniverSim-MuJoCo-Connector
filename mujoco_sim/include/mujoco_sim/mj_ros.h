// Copyright (c) 2022, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

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

#pragma once

#include "mj_sim.h"
#include "mujoco_msgs/DestroyObject.h"
#include "mujoco_msgs/ObjectInfo.h"
#include "mujoco_msgs/ObjectState.h"
#include "mujoco_msgs/ObjectStateArray.h"
#include "mujoco_msgs/ObjectStatus.h"
#include "mujoco_msgs/SpawnObject.h"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

enum EObjectType : std::int8_t
{
    None = 0,
    Robot = 1,
    World = 2,
    SpawnedObject = 3
};

class MjRos
{   
public:
    MjRos(const MjRos &) = delete;

    void operator=(MjRos const &) = delete;

    static MjRos &get_instance()
    {
        static MjRos mj_ros;
        return mj_ros;
    }

public:
    /**
     * @brief Set tmp_model_name, world_path and odom_joints
     *
     */
    static void set_params();

public:
    /**
     * @brief Initialize publishers, subscribers and MjRos members from rosparam
     *
     */
    void init();

    /**
     * @brief Setup service server threads
     *
     */
    void setup_service_servers();

    /**
     * @brief Get the controlled joints from ros_control
     *
     */
    void get_controlled_joints();

public:
    static ros::Time ros_start;
    
private:
    MjRos() = default; // Singleton

    ~MjRos();

private:
    void publish_sensor_data();

    void spawn_and_destroy_objects();

private:
    bool screenshot_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

    bool reset_robot_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

    bool spawn_objects_service(mujoco_msgs::SpawnObjectRequest &req, mujoco_msgs::SpawnObjectResponse &res);

    void spawn_objects(const std::vector<mujoco_msgs::ObjectStatus> objects);

    bool destroy_objects_service(mujoco_msgs::DestroyObjectRequest &req, mujoco_msgs::DestroyObjectResponse &res);

    void destroy_objects(const std::set<std::string> object_names);

    void add_object_state(const int body_id, const EObjectType object_type);

    void reset_robot();

private:
    ros::NodeHandle n;

    std::string root_frame_id;

    ros::ServiceServer screenshot_server;

    ros::ServiceServer reset_robot_server;

    ros::ServiceServer spawn_objects_server;

    ros::ServiceServer destroy_objects_server;

    std::map<std::string, float> joint_inits;
};