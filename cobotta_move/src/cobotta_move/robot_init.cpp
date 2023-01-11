#include "cobotta_move/robot_init.h"
#include "ros/ros.h"

robot_init::robot_init(int& loop,int& time,int& dof,robot_data& data,robot_states& states):
_time(time),
_loops(loop*time/2),
robot_move(loop,dof,data,states){
}
robot_init::robot_init(int& time,std::shared_ptr<robot_move>& base):
    robot_move(base),
    _time(time),
    _loops(_loop*time){
}

int robot_init::update(std_msgs::Float64MultiArray& out){
    loop_count++;
    calc_sin_value();
    for(int i=0;i<_dof;i++){
        out.data[i]=_data.first_pos.data[i]+sin_value*_data.start_diff.data[i];
    }
    if(loop_count==_loops){
        loop_count = 0;
        return 0;
    }
    return 1;
}
int robot_init::update(){
    loop_count++;
    calc_sin_value();
    for(int i=0;i<_dof;i++){
        _robot.joint_command.data[i]=_data.first_pos.data[i]+sin_value*_data.start_diff.data[i];
    };
    /*if(_dof==7){
        _robot.joint_command.data[6]=_data.first_pos.data[6]+sin_value*_data.start_diff.data[6];
    }*/
           //std::cout << "sin is  "<<sin_value << std::endl;
            
    if(loop_count>=_loops){
        _robot.joint_command.data = _data.start_pos.data;
        return 0;
    }
    return 1;
}

double robot_init::calc_sin_value(){
    sin_value=0.5*(1-cos(M_PI*loop_count/_loops));
    return sin_value;
}

