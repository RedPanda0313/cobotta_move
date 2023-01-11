#pragma once
#include "std_msgs/Float64MultiArray.h"

// data storage 
class robot_data{
    public:
    robot_data(const int& dof):_dof(dof){
        set_vals();
    }
    std_msgs::Float64MultiArray start_pos;//starting pose
    std_msgs::Float64MultiArray first_pos;//data holder for pos at start
    std_msgs::Float64MultiArray move_angle;
    std_msgs::Float64MultiArray start_diff; // diff in pos for initialize
    std_msgs::Float64MultiArray joint_max; // diff in pos for initialize
    std_msgs::Float64MultiArray joint_min; // diff in pos for initialize
    std_msgs::Float64MultiArray joint_speed; // diff in pos for initialize

    void init(std_msgs::Float64MultiArray &in);
    const int &_dof;
    std_msgs::Float64MultiArray get_start_pose();

private:
    void set_vals();
};