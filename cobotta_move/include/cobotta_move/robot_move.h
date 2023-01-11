#pragma once
#include "cobotta_move/robot_data.h"
#include "cobotta_move/robot_states.h"
#include <memory>

//base class for robot command generation
class robot_move{
    public:
    robot_move(int& loop,int& dof,robot_data& data,robot_states&states );
    robot_move(robot_move& copy);
    robot_move(std::shared_ptr<robot_move>& copy);
    void set_cobotta_curr(double (&in)[6]);
    void get_cobotta_comm(double (&out)[6]);

    virtual int update(std_msgs::Float64MultiArray& out){
        return -1;
    };
    virtual int update();
    void init();

    protected:
    const int& _dof;
    const int& _loop;
    robot_data& _data; 
    robot_states& _robot;
};