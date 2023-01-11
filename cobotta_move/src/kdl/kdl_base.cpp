#include "kdl/kdl_base.h"

kdl_base::kdl_base(int &dof, geometry_msgs::Pose& in,std_msgs::Float64MultiArray& out,robot_data& robot_param):
_dof(dof),
ros_in(in),
ros_out(out),
_robot_param(robot_param)
{
    q_dot.resize(_dof);
    q_pre.resize(_dof);
    q_out.resize(_dof);
    q_min.resize(_dof);
    q_max.resize(_dof);
    kdl_parse kdl_cobotta(_dof);
    chain=kdl_cobotta.chain();
    ik_ptr = std::unique_ptr<kdl_ik>(new kdl_ik(chain));
    
}
int kdl_base::update(){
    //conversion for pre
    r_in = KDL::Rotation::Quaternion(
        ros_in.orientation.x,
        ros_in.orientation.y,
        ros_in.orientation.y,
        ros_in.orientation.w);
    p_in = KDL::Vector(
        ros_in.position.x,
        ros_in.position.y, 
        ros_in.position.z);

    t_in = KDL::Frame(r_in, p_in);

    //ik calculation
    int ret = ik_ptr->ik_pos.CartToJnt(q_pre, t_in, q_out);
    // check 
    if(500>=ret&&ret>=0){
        for (int i = 0; i < _dof; i++)
        {
            ros_out.data[i] = q_out.data(i);
            q_pre.data = q_out.data;
        }
    // conversion for post
        
    }
    if(ret<0) {
    std::cout << ret << std::endl;
    }
    return ret;
}
geometry_msgs::Pose kdl_base::current_pose(const std_msgs::Float64MultiArray& now){
    for (int i = 0; i < _dof;i++){
        q_pre.data[i] = now.data[i];
        //std::cout<< now.data[i]<<" ";
    }
    KDL::Frame temp;
    ik_ptr->fk_pos.JntToCart(q_pre, temp );
    geometry_msgs::Pose ret;
    ret.position.x = temp.p.data[0];
    ret.position.y = temp.p.data[1];
    ret.position.z = temp.p.data[2];
    temp.M.GetQuaternion(
        ret.orientation.x,
        ret.orientation.y,
        ret.orientation.z,
        ret.orientation.w);
    return ret;
}
void kdl_base::set_limit(){
    ik_ptr->ik_pos.set_limit(_robot_param.joint_min, _robot_param.joint_max);
}