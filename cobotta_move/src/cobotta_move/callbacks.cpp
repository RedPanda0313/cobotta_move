#include "cobotta_move/callbacks.h"
std::mutex m;
callbacks::callbacks(robot_states &data,command_data &cmd) : _data(data), _dof(_data._dof),_cmd(cmd) {}

void callbacks::sim_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(m);
    _data.joint_now.data = msg->data;
    first_joint_call();
}

void callbacks::cobotta_cb(const sensor_msgs::JointState::ConstPtr& msg){
     std::lock_guard<std::mutex> lock(m);
    if(_dof==6){
        _data.joint_now.data=msg->position;
    }
    else{
        for(int i=0;i<6;i++){
            _data.joint_now.data[i]=msg->position[i];
        }
    }
    first_joint_call();
}

void callbacks::first_joint_call(){
    if(!first_joint){
        return;
    }
    else{
        first_joint=false;
        _data.recieved=true;
    }
}

void callbacks::leader_cb(const geometry_msgs::Pose::ConstPtr& msg){
    _cmd.command_recv.orientation.x =msg->orientation.x;
    _cmd.command_recv.orientation.y =msg->orientation.y;
    _cmd.command_recv.orientation.z =msg->orientation.z;
    _cmd.command_recv.orientation.w =msg->orientation.w;
    _cmd.command_recv.position.x =msg->position.x;
    _cmd.command_recv.position.y =msg->position.y;
    _cmd.command_recv.position.z =msg->position.z;
    _cmd.received = true;
}
void callbacks::clutch_cb(const std_msgs::Bool::ConstPtr& msg){
    _cmd.clutch.data = msg->data;
}