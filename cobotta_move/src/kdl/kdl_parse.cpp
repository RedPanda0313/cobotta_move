#include "kdl/kdl_parse.h"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include "ros/ros.h"
#include <map>
#include <resource_retriever/retriever.h>

kdl_parse::kdl_parse(int dof){
    dof_=dof;
    if (!model_.initFile("/home/remote-pc/catkin_ws/src/cobotta/urdf/cobotta_r3.urdf")){
      ROS_ERROR("bad urdf model");
      return ;
   }
   if (!kdl_parser::treeFromUrdfModel(model_, tree_)){
       ROS_ERROR("parse failed");
      return ;
   }
   get_chain();
}

void kdl_parse::get_chain(){
    
    if(dof_==6){
        tree_.getChain("base_link","J6",chain_);
        if(chain_.getNrOfJoints()==dof_){
            chain_is_good=true;
            ROS_INFO("6 dof KDL::Chain is ready");
            
        }

    }
    else if(dof_==7){
        tree_.getChain("base_link","Jtooltip",chain_);
        if(chain_.getNrOfJoints()==dof_){
            int nr = chain_.getNrOfSegments();
            chain_is_good = true;
            ROS_INFO("7 dof KDL::Chain is ready");
            ROS_INFO("chain has %d seg",nr);
        }
    }
    else {
        chain_is_good=false;
        ROS_ERROR("invalid DOF: %d",dof_);
    }
}

KDL::Chain kdl_parse::chain(){
    if(chain_is_good==false){
        ROS_ERROR("invalid DOF: %d",dof_);
        }
    return chain_;
}
