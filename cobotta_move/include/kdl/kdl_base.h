#pragma once
#include "kdl/kdl_parse.h"
#include "kdl/kdl_ik.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"

#include "cobotta_move/robot_data.h"

//base class for using kdl from outside
class kdl_base{
    public:
        kdl_base(int &dof, geometry_msgs::Pose& in,std_msgs::Float64MultiArray& out,robot_data& robot_param);
        int update();
        void start();
        geometry_msgs::Pose current_pose(const std_msgs::Float64MultiArray& in);//always call once
        void set_limit();

    protected:
        robot_data &_robot_param;
        KDL::Chain chain;
        int &_dof;
        std::unique_ptr<kdl_ik> ik_ptr;
        KDL::JntArray q_min;
        KDL::JntArray q_max;
        KDL::JntArray q_dot;
        KDL::JntArray q_out;//output
        KDL::JntArray q_pre;//calc start
        KDL::Frame t_in;//input
        KDL::Rotation r_in;//input
        KDL::Vector p_in;//input
        void init(robot_data& robot_param);
        geometry_msgs::Pose &ros_in;
        std_msgs::Float64MultiArray &ros_out;
};