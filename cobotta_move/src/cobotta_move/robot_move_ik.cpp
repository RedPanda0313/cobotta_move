#include "cobotta_move/robot_move_ik.h"

robot_move_ik::robot_move_ik(std::shared_ptr<robot_move>& base,command_data& cmds,int &dof ):
    robot_move(base),
    _cmds(cmds),
    _dof(dof),
    ik(dof,_cmds.command_pos,_robot.joint_command,_data)
    {
    }

int robot_move_ik::update(){
    if(!ik_ready){
    }
    else{
    //call ik
        switch (ik.update())
        {
            case 0:
            break;
            default:
            break;
        }

    }

}
