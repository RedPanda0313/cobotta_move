#include "cobotta_move/robot_move_ik_pose.h"
#include "cobotta_move/ik_pose_data.h"

robot_move_ik_pose::robot_move_ik_pose(int& time,std::shared_ptr<robot_move>& base,command_data& cmds,int& dof ):
robot_move_ik(base,cmds,dof),
_time(time)
{
    if(independent){
        loop_x = _loop * (_time + ik_pose::time_x - ik_pose::time);
        loop_y = _loop * (_time + ik_pose::time_y - ik_pose::time);
        loop_z = _loop * (_time + ik_pose::time_z - ik_pose::time);
    }
    loops = _loop * _time;
    ik.set_limit();
}
robot_move_ik_pose::robot_move_ik_pose(std::shared_ptr<robot_move>& base,command_data& cmds,int& dof ):
robot_move_ik(base,cmds,dof)
{
    _time = ik_pose::time;
    loops = _loop * _time;
    independent = true;
}
int robot_move_ik_pose::update()
{
std_msgs::Float64MultiArray temp;
    loop_count++;
    if (first)
    {

        calc_sin_value_init();

        if (loop_count == 1)
        {
            ROS_INFO("first");
            temp = _data.get_start_pose();
            start_pose = ik.current_pose(temp);
            _cmds.command_pos.orientation = start_pose.orientation;
        }
        if (independent)
        {
            _cmds.command_pos.position.x = start_pose.position.x + sin_value_init * ik_pose::amp_x / 1000;
            _cmds.command_pos.position.y = start_pose.position.y + sin_value_init * ik_pose::amp_y / 1000;
            _cmds.command_pos.position.z = start_pose.position.z + sin_value_init * ik_pose::amp_z / 1000;
        }
        else
        {
            _cmds.command_pos.position.x = start_pose.position.x + sin_value_init * ik_pose::amp/1000;
            _cmds.command_pos.position.y = start_pose.position.y + sin_value_init * ik_pose::amp/1000;
            _cmds.command_pos.position.z = start_pose.position.z + sin_value_init * ik_pose::amp/1000;
        }
            //std::cout << "sin is with "<<sin_value_init << std::endl;
            if(2.0*loop_count>=loops){
                first=false;
                loop_count=0;
                ROS_INFO("begin round-trip");
            }
        }
    else{
        calc_sin_value();
        if(independent){
            _cmds.command_pos.position.x = start_pose.position.x + sin_value_x * ik_pose::amp_x/1000;
            _cmds.command_pos.position.y = start_pose.position.y + sin_value_y * ik_pose::amp_y/1000;
            _cmds.command_pos.position.z = start_pose.position.z + sin_value_z * ik_pose::amp_z/1000;

        }
        else{
            _cmds.command_pos.position.x = start_pose.position.x + sin_value * ik_pose::amp/1000;
            _cmds.command_pos.position.y = start_pose.position.y + sin_value * ik_pose::amp/1000;
            _cmds.command_pos.position.z = start_pose.position.z + sin_value * ik_pose::amp/1000;
        }
    
    //_cmds.command_pos= start_pose;
    }
    
    //_cmds.command_pos= start_pose;
    int ret=ik.update();
    if(loop_count==1){

        /*std_msgs::Float64MultiArray temp1 = _robot.joint_command;
        geometry_msgs::Pose temp_pose = ik.current_pose(temp1);
        std::cout << temp.data[0] << " " << _robot.joint_command.data[0]<<std::endl;
        std::cout << temp.data[1] << " " << _robot.joint_command.data[1]<<std::endl;
        std::cout << temp.data[2] << " " << _robot.joint_command.data[2]<<std::endl;
        std::cout << temp.data[3] << " " << _robot.joint_command.data[3]<<std::endl;
        std::cout << temp.data[4] << " " << _robot.joint_command.data[4]<<std::endl;
        std::cout << temp.data[5] << " " << _robot.joint_command.data[5]<<std::endl;
        std::cout << _cmds.command_pos.orientation.x << " " << temp_pose.orientation.x<<std::endl;
        std::cout << _cmds.command_pos.orientation.y << " " << temp_pose.orientation.y<<std::endl;
        std::cout << _cmds.command_pos.orientation.z << " " << temp_pose.orientation.z<<std::endl;
        std::cout << _cmds.command_pos.orientation.w << " " << temp_pose.orientation.w<<std::endl;
        std::cout << _cmds.command_pos.position.x << " " << temp_pose.position.x<<std::endl;
        std::cout << _cmds.command_pos.position.y << " " << temp_pose.position.y<<std::endl;
        std::cout << _cmds.command_pos.position.z << " " << temp_pose.position.z<<std::endl;
        std::cout <<std::endl;*/
        
        
    }
    return ret;
}

double robot_move_ik_pose::calc_sin_value_init(){
    sin_value_init=0.5*(1-cos(M_PI*2*(loop_count)/loops));
    return sin_value_init;
}

double robot_move_ik_pose::calc_sin_value(){
    if(independent){
        sin_value_x=cos(M_PI*2*loop_count/loop_x); 
        sin_value_y=cos(M_PI*2*loop_count/loop_y); 
        sin_value_z=cos(M_PI*2*loop_count/loop_z); 
       /*sin_value_x=1; 
        sin_value_y=1; 
        sin_value_z=1; */
    }
    sin_value=cos(M_PI*2*loop_count/loops);
    return sin_value;
}
        