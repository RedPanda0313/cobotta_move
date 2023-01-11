#include "cobotta_move/robot_rviz.h"

rviz_joint_msg::rviz_joint_msg(int dof,ros::NodeHandle* nh){
    _dof = dof;
    out.header.stamp = ros::Time::now();
    out.name.resize(7);
    out.position.resize(7,0.0);
    out.name[0] = "joint_1";
    out.name[1] = "joint_2";
    out.name[2] = "joint_3";
    out.name[3] = "joint_4";
    out.name[4] = "joint_5";
    out.name[5] = "joint_6";
    out.name[6] = "joint_tip";
    pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);
}

void rviz_joint_msg::update(const std_msgs::Float64MultiArray& in){
    
    out.header.stamp = ros::Time::now();
    for (int i = 0; i < 6;i++){
        out.position[i] = in.data[i];
    }
    if(_dof==7){
        out.position[6] = in.data[6];
    }
    pub.publish(out);
}