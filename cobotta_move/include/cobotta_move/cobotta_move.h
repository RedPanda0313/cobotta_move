#pragma once
//my header
#include "cobotta_move/callbacks.h"
#include "cobotta_move/robot_states.h"
#include "cobotta_move/robot_init.h"
#include "cobotta_move/robot_move.h"
#include "cobotta_move/robot_move_joint.h"
#include "cobotta_move/robot_move_lf.h"
#include "cobotta_move/robot_move_ik.h"
#include "cobotta_move/robot_move_ik_pose.h"
#include "cobotta_move/robot_move_ik_roll.h"
#include "cobotta_move/robot_rviz.h"
#include "cobotta_move/setting_data.h"
#include "cobotta_move/command_data.h"
//others
#include "ros/ros.h"
#include <memory>

//moves cobotta
class cobotta_move{
    public:
    cobotta_move(setting_data& setting,ros::NodeHandle* nh);
    cobotta_move(setting_data& setting,robot_data&,robot_states&, callbacks&,ros::NodeHandle* nh);
    int update();
    void cobotta_joint_in(double(&in)[6]);
    void cobotta_joint_out(double(&out)[6]);
    void tool_in(double & in);
    void tool_out(double & out);

    void set_no_pub(){
        should_pub = false;
    }
    void set_pub(){
        should_pub = true;
    }
    void no_sub(){
        sub_joint.shutdown();
    }
    robot_data const_data;
    robot_states data_now;
    callbacks cb;
    command_data lf_command;

protected:
    ros::NodeHandle pnh;
    setting_data& settings;
    std::shared_ptr<robot_move> command_base;

    private:
        bool both = true;
        ros::Subscriber sub_joint;
        ros::Subscriber sub_cmd;
        ros::Subscriber sub_clutch;
        ros::Publisher pub_joint;
        ros::Publisher pub_tool;
        ros::Publisher pub_sim_;
        ros::Publisher pub_real_;
        void init(ros::NodeHandle *nh);
        int data_ready();
        int move_to_start();
        int move();
        bool ready = false;
        bool to_start = true;
        int result = 0;
        std::unique_ptr<robot_init> command_init;
        std::unique_ptr<robot_move> command_move;
        bool should_pub = true;
        std::unique_ptr<rviz_joint_msg> rviz;
};