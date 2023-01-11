#include "cobotta_move/robot_move_joint.h"
#include "ros/ros.h"

robot_move_joint::robot_move_joint(int& time,std::shared_ptr<robot_move>& base ):
    robot_move(base),
    _time(time),
    _loops(_loop*time)
    {
}

int robot_move_joint::update(){
    loop_count++;
    if(first){
        calc_sin_value_init();

        for(int i=0;i<6;i++){
            _robot.joint_command.data[i]=_data.start_pos.data[i]+sin_value_init*_data.move_angle.data[i];
        }
        if(_dof==7){
            _robot.tool_command.data=_data.start_pos.data[6]+sin_value_init*_data.move_angle.data[6];
        }
        if(loop_count*2>=_loops){
            first=false;
            loop_count=0;
            ROS_INFO("begin round-trip");
        }
    }
    else{
        calc_sin_value();

        for(int i=0;i<6;i++){
            _robot.joint_command.data[i]=_data.start_pos.data[i]+sin_value*_data.move_angle.data[i];
        }
        if(_dof==7){
            _robot.tool_command.data=_data.start_pos.data[6]+sin_value*_data.move_angle.data[6];
        }

    }
    return 1;
}

double robot_move_joint::calc_sin_value_init(){
    sin_value_init=0.5*(1-cos(M_PI*2*loop_count/_loops));
    return sin_value_init;
}
double robot_move_joint::calc_sin_value(){
    sin_value=cos(M_PI*2*loop_count/_loops);
    //sin_value=1;
    //bool rot = sin_value > 0 ? true : false;
    //ROS_INFO("%d", rot);
    return sin_value;
}