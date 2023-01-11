#include "cobotta_move/robot_move.h"


robot_move::robot_move(int& loop,int& dof,robot_data& data,robot_states& states):
_loop(loop),
_dof(dof),
_data(data),
_robot(states){

}
robot_move::robot_move(robot_move& copy):
_dof(copy._dof),
    _loop(copy._loop),
    _data(copy._data),
    _robot(copy._robot){
    
}

robot_move::robot_move(std::shared_ptr<robot_move>& copy):
    _dof(copy->_dof),
    _loop(copy->_loop),
    _data(copy->_data),
    _robot(copy->_robot){
    
}
int robot_move::update(){
    _robot.joint_command=_robot.joint_now;
    return 1;
}
void robot_move::init(){
    _data.init(_robot.joint_now);
}
void robot_move::set_cobotta_curr(double(&in)[6]){
    for (int i = 0; i < 6;i++){
        _robot.joint_now.data[i] = in[i];
    }
    _robot.recieved= true;
}

void robot_move::get_cobotta_comm(double(&out)[6]){
    for (int i = 0; i < 6;i++){
        out[i] = _robot.joint_command.data[i];
    }
}