#pragma once
#include "cobotta_move/robot_move_ik.h"

//move pos in sin wave with ik
class robot_move_ik_pose:public robot_move_ik{
    public:
        robot_move_ik_pose(int& time,std::shared_ptr<robot_move>& base,command_data& cmds,int& dof );
        robot_move_ik_pose(std::shared_ptr<robot_move>& base,command_data& cmds,int& dof );
        int update();

        private:
            bool independent = true;
            int _time;
            float loop_x;
            float loop_y;
            float loop_z;
            float loops;
            float amp;
            int loop_count = 0;
            double calc_sin_value();
            double calc_sin_value_init();
            double sin_value;
            double sin_value_init;
            double sin_value_x;
            double sin_value_y;
            double sin_value_z;
            bool first = true;
            geometry_msgs::Pose start_pose;
            void set_limit();
};