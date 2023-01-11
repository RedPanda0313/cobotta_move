#include"cobotta_move/cobotta_param.h"

setting_data getparam(){
    setting_data out;
    ros::NodeHandle pnh("~");

    //get params
    pnh.getParam("dof",out.dof);
    pnh.getParam("hz",out.node_looprate);
    pnh.getParam("sim",out.sim);
    pnh.getParam("init_time",out.init_time);
    pnh.getParam("loop_time",out.loop_time);
    pnh.getParam("mode",out.op_mode);
    return out;

}
setting_data getparam(ros::NodeHandle& pnh){
    setting_data out;
    //get params
    
    pnh.getParam("hz",out.node_looprate);
    pnh.getParam("sim",out.sim);
    pnh.getParam("init_time",out.init_time);
    pnh.getParam("loop_time",out.loop_time);
    pnh.getParam("mode",out.op_mode);
    return out;

}

void getparam(setting_data& out)
{
       
    ros::NodeHandle pnh("~");
    pnh.getParam("dof",out.dof);
    pnh.getParam("hz",out.node_looprate);
    pnh.getParam("sim",out.sim);
    pnh.getParam("init_time",out.init_time);
    pnh.getParam("loop_time",out.loop_time);
    pnh.getParam("mode",out.op_mode);
ROS_INFO("here") ;
}
void getparam(setting_data& out,ros::NodeHandle& pnh){
    pnh.getParam("dof",out.dof);
    pnh.getParam("hz",out.node_looprate);
    pnh.getParam("sim",out.sim);
    pnh.getParam("init_time",out.init_time);
    pnh.getParam("loop_time",out.loop_time);
    pnh.getParam("mode",out.op_mode);
}