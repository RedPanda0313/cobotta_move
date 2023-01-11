#pragma once
#include"cobotta_move/setting_data.h"

#include"ros/ros.h"

//getting parameters from ros 
setting_data getparam();
setting_data getparam(ros::NodeHandle& pnh);
void getparam(setting_data& setting,ros::NodeHandle& pnh);
void getparam(setting_data& setting);

