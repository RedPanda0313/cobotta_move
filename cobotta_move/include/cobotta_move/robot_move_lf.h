#pragma once
#include "cobotta_move/robot_move_ik.h"
#include "cobotta_move/command_data.h"

//class for leader folllower
class robot_move_lf: public robot_move_ik{
    public:
    robot_move_lf(std::shared_ptr<robot_move>& base,command_data& cmds,int& dof );
    //int update(std_msgs::Float64MultiArray& out);
    int update();
    
    protected:
        int calc();
        int &_dof;
        bool aligns();
        bool align = true;
        geometry_msgs::Pose first_pose_follower; // start position of actual robot
        geometry_msgs::Pose pre_pose_leader;     // last pose of leader
        geometry_msgs::Pose pre_pose_command;    // last pose of robot before clutch on;
        bool first = true;
        bool waiting = true;
        ;
        bool pre_clutch=true; //previous loop clutch state 
        int ms = 1;//motion scaling
};