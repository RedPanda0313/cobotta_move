#pragma once
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_my.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/kdl_fk.h>

//class for using ik with kdl. easily change method here
class kdl_ik{
    public:
        kdl_ik(KDL::Chain chain);
        kdl_ik(KDL::Chain chain,KDL::JntArray q_min,KDL::JntArray q_max);
        KDL::ChainFkSolverPos_recursive fk_pos;
        KDL::ChainIkSolverVel_pinv ik_vel;
        KDL::ChainIkSolverPos_MY ik_pos;
        int update();
        int i;
};