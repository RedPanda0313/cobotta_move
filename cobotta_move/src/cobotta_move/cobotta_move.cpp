#include"cobotta_move/cobotta_move.h"
#include"cobotta_move/cobotta_param.h"

#ifndef USE_SIM_AND_REAL
    //#define USE_SIM_AND_REAL
#endif
cobotta_move::cobotta_move( setting_data &setting_,robot_data& data_,robot_states& states_, callbacks& cb_,ros::NodeHandle* nh):
    settings(setting_),
    const_data(data_),
    data_now(states_),
    cb(cb_){
        init(nh);
}

cobotta_move::cobotta_move( setting_data &setting,ros::NodeHandle* nh):
    settings(setting),
    const_data(robot_data(setting.dof)),
    data_now(robot_states(setting.dof)),
    cb(callbacks(data_now,lf_command))
    {
        init(nh);
}

void cobotta_move::init(ros::NodeHandle* node){
    to_start = settings.to_start;
    command_base = std::shared_ptr<robot_move>(new robot_move(settings.node_looprate, settings.dof, const_data, data_now));
    command_init=std::unique_ptr<robot_init>(new robot_init(settings.init_time,command_base));
    
    switch(settings.op_mode){
        case 0://joint move in sin wave

            ROS_INFO("moving by angle");
            command_move = std::unique_ptr<robot_move_joint>(new robot_move_joint(settings.loop_time, command_base));
            break;
        case 1: // lf
            ROS_INFO("moving by leader-follower");
            command_move = std::unique_ptr<robot_move_lf>(new robot_move_lf( command_base, lf_command,settings.dof));
            sub_cmd = node->subscribe("/Leader_Right", 1, &callbacks::leader_cb, &cb);
            sub_clutch = node->subscribe("/Clutch", 1, &callbacks::clutch_cb, &cb);
            break;
        case 2: //ik only
            ROS_INFO("moving by ik (position)");
            command_move = std::unique_ptr<robot_move_ik_pose>(new robot_move_ik_pose(settings.loop_time, command_base, lf_command, settings.dof));
            break;
        case 3: //ik only
            ROS_INFO("moving by ik (orientation)");
            command_move = std::unique_ptr<robot_move_ik_roll>(new robot_move_ik_roll(settings.loop_time, command_base, lf_command, settings.dof));
            break;
        }

    if(!settings.sim) sub_joint=node->subscribe("cobotta_r/joint_states",1,&callbacks::cobotta_cb,&cb);
    else sub_joint=node->subscribe("/joints_from_sim",1,&callbacks::sim_cb,&cb);
    
#ifndef USE_SIM_AND_REAL  
   //sub_joint=node->subscribe("cobotta_r/joint_states",1,&callbacks::cobotta_cb,&cb);
    if(!settings.sim) pub_joint=node->advertise<std_msgs::Float64MultiArray> ("/cobotta_r/arm_controller2/command",1);
    else{
        pub_joint=node->advertise<std_msgs::Float64MultiArray>("/cobotta_r/arm_controller2/sim",1);
        rviz=std::unique_ptr<rviz_joint_msg>(new rviz_joint_msg(settings.dof, node));
    }
#else
        pub_real_=node->advertise<std_msgs::Float64MultiArray> ("/cobotta_r/arm_controller2/command",1);
        pub_sim_=node->advertise<std_msgs::Float64MultiArray> ("/cobotta_r/arm_controller2/sim",1);
#endif
     
    if(settings.sim){
        //settings.init_time = 4;
            command_base->init();
    }    
    }
    
int cobotta_move::update(){
    int ret = -1000;
    if (settings.sim || data_ready() == 1)
    {
            if (to_start)
            {
                ret=move_to_start();
            }
            else{
            ret=move();
        }
        if(should_pub){
#ifndef USE_SIM_AND_REAL  
            if(settings.sim){

                pub_joint.publish(data_now.joint_command);
                rviz->update(data_now.joint_command);
            }
            else{
                data_now.separate();
                pub_joint.publish(data_now.cobotta_command);
            }
#else
            data_now.separate();
            pub_real_.publish(data_now.cobotta_command);
            pub_sim_.publish(data_now.joint_command);
#endif
        }
    }
    return result;
}
int cobotta_move::data_ready(){
    if(ready){
        return 1;
    }
    if(data_now.is_ready()){
        //value initialization
        ready=true;

        if(to_start){
            command_base->init();
        }
        ROS_INFO("ready to start");
        return 1;
    }
    return 0;
}
int cobotta_move::move_to_start(){
    result=command_init->update();
    if(result==0){
        to_start=false;
        ROS_INFO("at start");
    }
    return 1;
}
int cobotta_move::move(){
    
    result=command_move->update();
}

void cobotta_move::cobotta_joint_in(double(&in)[6]){
    command_base->set_cobotta_curr(in);
}
void cobotta_move::cobotta_joint_out(double(&out)[6]){
    command_base->get_cobotta_comm(out);
}