#include "cobotta_move/robot_move_lf.h"
#include "utility/ros_calc.h"

robot_move_lf::robot_move_lf(std::shared_ptr<robot_move> &base, command_data &cmds, int &dof) : robot_move_ik(base, cmds, dof),
                                                                                                _dof(dof)
{
    ik.set_limit();
    ms = 10;
}

int robot_move_lf::update()
{
    int ret = 0;

    if (!_cmds.received && waiting)
    {
        if (waiting)
        {
            ROS_INFO("waiting for leader device");
            waiting = false;

        } // check
    }
    else
    {
        if (first)
        {

            std_msgs::Float64MultiArray temp;
            geometry_msgs::Pose temp1;
            temp = _data.get_start_pose();
            pre_pose_command = ik.current_pose(temp);
            pre_pose_leader = _cmds.command_recv;
            _cmds.command_pos = pre_pose_command;
            first = false;
            //std::cout << _cmds.command_pos.position.z << std::endl;
        }

        else 
        {
            if (calc())
            {
                if (_cmds.command_pos.position.z < 0.05)
                {

                    _cmds.command_pos.position.z = 0.05; // prevent collision
                }
                ret = ik.update();
                //std::cout << _cmds.command_pos.position.z << std::endl;
            }
        }
    }

    return ret;
}
int robot_move_lf::calc()
{
    int ret = 0;
    if (!_cmds.clutch.data)
    {
        if (pre_clutch)
        {
            pre_pose_leader = _cmds.command_recv;
            ROS_INFO("released clutch");
        }
        //_cmds.command_pos.orientation = utility::multiply_orientation(pre_pose_command,utility::multiply_orientation(utility::inverse_orientation(pre_pose_leader),_cmds.command_recv)).orientation;
        _cmds.command_pos.position.x = pre_pose_command.position.x - (pre_pose_leader.position.x - _cmds.command_recv.position.x) / (1000 * ms);
        _cmds.command_pos.position.y = pre_pose_command.position.y - (pre_pose_leader.position.y - _cmds.command_recv.position.y) / (1000 * ms);
        _cmds.command_pos.position.z = pre_pose_command.position.z - (pre_pose_leader.position.z - _cmds.command_recv.position.z) / (1000 * ms);
        _cmds.command_pos.orientation = _cmds.command_recv.orientation;
        ret = 1;
    }
    else if (!pre_clutch)
    {
        pre_pose_command = _cmds.command_pos;
        ROS_INFO("stepped clutch");
    }
    pre_clutch = _cmds.clutch.data;
    return 1;
}
