#pragma once
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

class rviz_joint_msg{
    public:
        rviz_joint_msg(int dof,ros::NodeHandle* nh);
        void update(const std_msgs::Float64MultiArray& in);
    
    private:
        int _dof;
        ros::Publisher pub;
        sensor_msgs::JointState out;
};