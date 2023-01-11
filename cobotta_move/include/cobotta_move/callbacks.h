#pragma once
#include "cobotta_move/robot_states.h"
#include "cobotta_move/command_data.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include <mutex>

//ros callbacks
class callbacks{
    public:
    callbacks(robot_states& data,command_data& cmd);
    void sim_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void cobotta_cb(const sensor_msgs::JointState::ConstPtr& msg);
    void leader_cb(const geometry_msgs::Pose::ConstPtr& msg);
    void tool_cb(const std_msgs::Float64::ConstPtr& msg);
    void clutch_cb(const std_msgs::Bool::ConstPtr& msg);
    
    private:

    robot_states& _data;
    command_data &_cmd;
    const int &_dof;
    bool first_joint=true;
    void first_joint_call();
};