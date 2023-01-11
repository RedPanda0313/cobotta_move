#include "cobotta_move/robot_states.h"

robot_states::robot_states(const int& dof):_dof(dof){
    joint_command.data.resize(_dof,0.0);
    cobotta_command.data.resize(6,0.0);
    joint_now.data.resize(_dof,0.0);
    tool_command.data=0.0;
}
void robot_states::separate(){
    for (int i = 0; i < _dof;i++){
            cobotta_command.data[i] = joint_command.data[i];
        }
}