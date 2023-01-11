#include"kdl/kdl_ik.h"

kdl_ik::kdl_ik(KDL::Chain chain):
fk_pos(KDL::ChainFkSolverPos_recursive(chain)),
ik_vel(KDL::ChainIkSolverVel_pinv(chain)),
ik_pos(KDL::ChainIkSolverPos_MY(chain,fk_pos,ik_vel,100,10e-9)){

}


int kdl_ik::update(){
    //return ik_pos->CartToJnt();
}