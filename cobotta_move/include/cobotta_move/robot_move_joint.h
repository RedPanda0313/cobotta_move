#pragma once
#include "cobotta_move/robot_move.h"

//moves all joints in sin
class robot_move_joint: public robot_move{
    public:
    robot_move_joint(std::shared_ptr<robot_move>& base );
    robot_move_joint(int& time,std::shared_ptr<robot_move>& base );
    int update(int loop);
    //int update(std_msgs::Float64MultiArray& out);
    int update();
    int first_quater();
    
    private:
    const int& _time;
    const int _loops;

    private:
    int loop_count=0;
    double calc_sin_value();
    double calc_sin_value_init();
    double sin_value;
    double sin_value_init;
    bool first=true;
};