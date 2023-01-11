#pragma once
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include "ros/ros.h"
#include <resource_retriever/retriever.h>

// kdl model extraction from urdf
class kdl_parse{
    public:
    kdl_parse(int dof);
    void get_chain();
    KDL::Chain chain();
    protected:
    KDL::Tree tree_;
    KDL::Chain chain_;
    urdf::Model model_;
    int dof_;
    bool chain_is_good=false;
};