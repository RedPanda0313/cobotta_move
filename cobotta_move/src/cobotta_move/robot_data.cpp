#include "cobotta_move/robot_data.h"

void robot_data::set_vals(){
        start_pos.data.resize(_dof,0.0);
        first_pos.data.resize(_dof,0.0);
        start_diff.data.resize(_dof,0.0);
        move_angle.data.resize(_dof,0.0);
        joint_max.data.resize(_dof,0.0);
        joint_min.data.resize(_dof,0.0);

        start_pos.data[0]=-50.0*M_PI/180;
        start_pos.data[1]=-15.0*M_PI/180;
        start_pos.data[2]=85.0*M_PI/180;
        start_pos.data[3]=-30.0*M_PI/180;
        start_pos.data[4]=30.0*M_PI/180;
        start_pos.data[5]=-15.0*M_PI/180;

        move_angle.data[0]=M_PI/180*15;
        move_angle.data[1]=M_PI/180*15;
        move_angle.data[2]=-M_PI/180*15;
        move_angle.data[3]=-M_PI/180*15;
        move_angle.data[4]=M_PI/180*15;
        move_angle.data[5]=M_PI/180*15;

        joint_max.data[0] = M_PI/180*150;
        joint_max.data[1] = M_PI/180*100;
        joint_max.data[2] = M_PI/180*140;
        joint_max.data[3] = M_PI/180*170;
        joint_max.data[4] = M_PI/180*135;
        joint_max.data[5] = M_PI/180*170;

        joint_min.data[0] = -M_PI/180*150;
        joint_min.data[1] = -M_PI/180*60;
        joint_min.data[2] = M_PI/180*18;
        joint_min.data[3] = -M_PI/180*170;
        joint_min.data[4] = -M_PI/180*95;
        joint_min.data[5] = -M_PI/180*170;

        if(_dof==7){
            start_pos.data[6]=M_PI/180*0;
            move_angle.data[6]=M_PI/180*0;
            joint_min.data[6] = -M_PI/180*6000;
            joint_max.data[6] = M_PI/180*6000;

        }
        for (int i = 0; i < _dof;i++){
            start_pos.data[i] = (joint_max.data[i] + joint_min.data[i]) / 2;
        }
        if(_dof==7){
            start_pos.data[6]=M_PI/180*80;
            move_angle.data[6]=M_PI/180*0;
            joint_min.data[6] = -M_PI/180*6000;
            joint_max.data[6] = M_PI/180*6000;

        }
        /*start_pos.data[0] += M_PI *( -65) / 180;
        start_pos.data[1] += M_PI *(10) / 180;
        start_pos.data[2] += M_PI * (25) / 180;
        start_pos.data[3] += M_PI * (-35) / 180;
        start_pos.data[4] += M_PI * (15) / 180;
        start_pos.data[5] += M_PI * (60) / 180;*/

        //worls on 7dof
        /*start_pos.data[0] += M_PI *( 50) / 180;
        start_pos.data[1] += M_PI *(-20) / 180;
        start_pos.data[2] += M_PI * (32) / 180;
        start_pos.data[3] += M_PI * (-20) / 180;
        start_pos.data[4]+= M_PI * (-80) / 180;
        start_pos.data[5] += M_PI * (80) / 180;*/
        
        start_pos.data[0] += M_PI *( 30) / 180;
        start_pos.data[1] += M_PI *(-20) / 180;
        start_pos.data[2] += M_PI * (32) / 180;
        start_pos.data[3] += M_PI * (47) / 180;
        start_pos.data[4]+= M_PI * (-85) / 180;
        start_pos.data[5] += M_PI * (100) / 180;
}
    void robot_data::init(std_msgs::Float64MultiArray& in){
        first_pos=in;
        for(int i=0;i<_dof;i++){
            start_diff.data[i]=start_pos.data[i]-first_pos.data[i];
        }
    }
std_msgs::Float64MultiArray robot_data::get_start_pose(){
    std_msgs::Float64MultiArray ret;
    ret.data.resize(_dof,0.0);
    for (int i = 0; i < _dof;i++){
        ret.data[i] = start_pos.data[i];
    }
    return ret;
}