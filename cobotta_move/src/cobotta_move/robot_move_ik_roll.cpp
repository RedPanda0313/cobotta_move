#include "cobotta_move/robot_move_ik_roll.h"
#include "cobotta_move/ik_pose_data.h"
#include "utility/ros_calc.h"

robot_move_ik_roll::robot_move_ik_roll(int& time,std::shared_ptr<robot_move>& base,command_data& cmds,int& dof ):
robot_move_ik(base,cmds,dof),
_time(time)
{
    
    loops = _loop * _time;
    ik.set_limit();
}
robot_move_ik_roll::robot_move_ik_roll(std::shared_ptr<robot_move>& base,command_data& cmds,int& dof ):
robot_move_ik(base,cmds,dof)
{
    _time = ik_pose::time;
    loops = _loop * _time;
}
int robot_move_ik_roll::update()
{
    std_msgs::Float64MultiArray temp;
    loop_count++;
    if (loop_count < 500){
        ROS_INFO("wait");
        return 0;
    }
    else if(loop_count==500)
    {
        ROS_INFO("first");
        temp = _data.get_start_pose();
        start_pose = ik.current_pose(temp);
        _cmds.command_pos = start_pose;
    }
    calc_theta();
    calc_quaternion();
    _cmds.command_pos.orientation = utility::multiply_orientation(pose_change,start_pose).orientation;
   // _cmds.command_pos.position.x = start_pose.position.x + 0.01 * sine;
   // _cmds.command_pos.position.y = start_pose.position.y + 0.01 * sine;
   // _cmds.command_pos.position.z = start_pose.position.z + 0.01 * sine;
    // std::cout << _cmds.command_pos.orientation.w << " " << pose_change.orientation.w << std::endl;

    int ret = ik.update();
    return ret;
}

double robot_move_ik_roll::calc_theta(){
    sine = sin(M_PI * 2 * (loop_count-500) / loops);
    theta = M_PI / 180 * sine * roll_deg;
    //std::cout << theta << std::endl;
    return theta;
}

void robot_move_ik_roll::calc_quaternion(){
    pose_change.orientation.z = sqrt(2)*sin(theta / 2);
    pose_change.orientation.y = sqrt(2)*sin(theta / 2);
    pose_change.orientation.w = cos(theta / 2);
}
        