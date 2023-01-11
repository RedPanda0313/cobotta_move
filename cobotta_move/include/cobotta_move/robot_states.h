#pragma once
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

//data of robots current states
class robot_states
{
    public:
    robot_states(const int& dof);
    bool is_ready(){
        return recieved;
    }
    bool recieved=false;
    void copy_data();
    void separate();
    // data
    std_msgs::Float64MultiArray joint_now;//size: dof
    std_msgs::Float64MultiArray joint_command;//size: dof
    std_msgs::Float64MultiArray cobotta_command;//size: 6
    std_msgs::Float64 tool_command;//size: 1
    // type target_pose;
    //type tool
    const int& _dof;

};
