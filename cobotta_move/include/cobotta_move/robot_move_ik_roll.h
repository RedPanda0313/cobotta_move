#pragma once
#include "cobotta_move/robot_move_ik.h"

//move pos in sin wave with ik
class robot_move_ik_roll:public robot_move_ik{
    public:
        robot_move_ik_roll(int& time,std::shared_ptr<robot_move>& base,command_data& cmds,int& dof );
        robot_move_ik_roll(std::shared_ptr<robot_move>& base,command_data& cmds,int& dof );
        int update();

        private:
            int _time;
            float loops;
            float amp;
            int loop_count = 0;
            double calc_theta();
            double theta;
            double sine;
            bool first = true;
            geometry_msgs::Pose start_pose;
            geometry_msgs::Pose pose_change;
            void set_limit();
            float roll_deg = 15;
            void calc_quaternion();
};