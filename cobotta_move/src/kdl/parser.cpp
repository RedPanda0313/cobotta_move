 #include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include "ros/ros.h"
#include <map>
#include <resource_retriever/retriever.h>
int main(int argc, char** argv){
    KDL::Tree my_tree;

   urdf::Model my_model;
   if (!my_model.initFile("/home/amd/catkin_ws4/src/cobotta_model/urdf/cobotta_r.urdf")){
      return false;
   }
   if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
      return false;
   }
   
   std::map<std::string,KDL::TreeElement> segment=my_tree.getSegments();
   for(auto itr=segment.begin();itr!=segment.end();++itr){
       std::string link_name = itr->first;
       std::cout<<link_name<<std::endl;
   }
   
  KDL::Chain chain;
  my_tree.getChain("base_link","Jtooltip",chain);
  int num;
  num=chain.getNrOfJoints();
  ROS_INFO("num of joint is ...%d",num);



}

