#pragma once
/*#include "cobotta_move/robot_data.h"
#include "cobotta_move/robot_states.h"*/
#include "cobotta_move/robot_move.h"
#include <cmath>
#define _USE_MATH_DEFINES

/*class robot_init{
    public:
    robot_init(int& loop,int& time,int& dof,robot_data& data,robot_states&robot );
    int update(int loop);
    int update(std_msgs::Float64MultiArray& out);
    int update();
    private:
    const int& _dof;
    const int& _loop;
    const int& _time;
    const int _loops;
    private:
    int loop_count=0;
    double calc_sin_value();
    double sin_value;
    robot_data& _data; 
    robot_states& robot; 
};*/
//for moving robot to starting posiiton
class robot_init: public robot_move{
    public:
    robot_init(int& loop,int& time,int& dof,robot_data& data,robot_states&robot );//full constructor
    robot_init(int& time,std::shared_ptr<robot_move>& base);//copy constructor
    int update(int loop);
    int update(std_msgs::Float64MultiArray& out);
    int update();
    
    private:
    const int& _time;
    const int _loops;

    private:
    int loop_count=0;
    double calc_sin_value();
    double sin_value;
};