//custom
#include "kdl/kdl_parse.h"
#include "cobotta_move/callbacks.h"
#include "cobotta_move/robot_states.h"
#include "cobotta_move/robot_init.h"
#include "cobotta_move/robot_move.h"
#include "cobotta_move/robot_move_joint.h"
#include "cobotta_move/robot_move_lf.h"
//others
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <memory>

int main(int argc, char* argv[]){

    //setting vals (defines the movements)
    int node_looprate=125;
    int dof=6;
    bool sim=true;
    int op_mode=0;
    int init_time=60;
    int loop_time=60;
    bool to_start=true;

    //internal vals (do not change)
    bool ready=false;
    int result=0;


    //ROS initialiaization
    ros::init(argc,argv,"cobotta_command_node");
    ros::NodeHandle node;
    ros::NodeHandle pnh("~");

    //get params
    pnh.getParam("dof",dof);
    pnh.getParam("hz",node_looprate);
    pnh.getParam("sim",sim);
    pnh.getParam("init_time",init_time);
    pnh.getParam("loop_time",loop_time);
    ros::Rate loop_rate(node_looprate);

    // data management
    command_data cmd;
    robot_data const_data(dof); // setting values
    robot_states data(dof);//control values
    callbacks cb(data,cmd);//callback functions

    //subs and pubs
    ros::Subscriber sub_joint;
    /*if(!sim) sub_joint=node.subscribe("cobotta_r/joint_states",1,&callbacks::cobotta_cb,&cb);
    else sub_joint=node.subscribe("/joints_from_sim",1,&callbacks::sim_cb,&cb);
*/sub_joint=node.subscribe("cobotta_r/joint_states",1,&callbacks::cobotta_cb,&cb);

    ros::Publisher pub_joint;
    if(!sim) pub_joint=node.advertise<std_msgs::Float64MultiArray> ("/cobotta_r/arm_controller2/command",1);
    else pub_joint=node.advertise<std_msgs::Float64MultiArray>("/cobotta_r/arm_controller2/sim",1);
    
    //kinematics with kdl
    kdl_parse kdl_cobotta(dof);

    //command generators
        //base unit for data managment
    std::shared_ptr<robot_move> command_base(new robot_move(node_looprate,dof,const_data,data));
        //init unit 
    std::unique_ptr<robot_init> command_init(new robot_init(init_time,command_base));
    
        //main unit 
    std::unique_ptr<robot_move> command_move;//not initialized
    if(1){
        command_move=std::unique_ptr<robot_move_joint> (new robot_move_joint(loop_time,command_base));
    }

    //robot_move* command_base=new robot_move(node_looprate,dof,const_data,data);
    //robot_init* command_init=new robot_init(node_looprate,init_time,dof,const_data,data);
    //robot_init* command_init2=new robot_init(init_time,command_base);
    //std::shared_ptr<robot_move> command_move2(new robot_move(node_looprate,dof,const_data,data));
    //robot_move* command_move=command_base;//temp
    //command_move=new robot_init(node_looprate,init_time,dof,const_data,data);


    //main loop
    while(ros::ok()){
        //command generation
        if(data.is_ready()){
            //move to start position
            if(to_start){
                result=command_init->update();
                if(result==0){
                    to_start=false;
                    ROS_INFO("at start");
                    const_data.init(data.joint_now);
                }else if(result==-1){
                    break;
                }
            }
            //specific command generation
            else{
                result=command_move->update();
                 if(result==-1){
                    break;
                }
            }
        }
        //pub data if ready
        if(ready){
            pub_joint.publish(data.joint_command);
        }
        //check if comm. is ready
        else{
            if(data.is_ready()){
                //value initialization
                ready=true;
                ROS_INFO("ready to start");
                const_data.init(data.joint_now);
            }
            
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}