#pragma once
#include <kdl/chainfksolverpos_recursive.hpp>


//not used
class kdl_fk{
    public:
        kdl_fk(KDL::Chain chain):fk_pos(chain){
            //fk constructor
        }
        KDL::ChainFkSolverPos_recursive fk_pos;
};