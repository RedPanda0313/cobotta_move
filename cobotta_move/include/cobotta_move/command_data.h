#pragma once
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"

//data received from leader
class command_data{
    public:
    command_data(){
        clutch.data = true;
    }
        geometry_msgs::Pose command_pos;
        geometry_msgs::Pose command_recv;
        bool received = false;
        std_msgs::Bool clutch;
};