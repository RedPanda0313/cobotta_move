#pragma once
#include "cobotta_move/robot_move.h"
#include "cobotta_move/command_data.h"
#include "kdl/kdl_base.h"

//base class for robot ik
class robot_move_ik: public robot_move{
    public:
    robot_move_ik(std::shared_ptr<robot_move>& base );
    robot_move_ik(std::shared_ptr<robot_move>& base,command_data& cmds,int& dof );
    int update();
    
    protected:
        command_data &_cmds;  
        kdl_base ik;
        int _dof;
        bool ik_ready = false;

    private:
};