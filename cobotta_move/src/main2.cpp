#include "cobotta_move/cobotta_move.h"
#include "cobotta_move/cobotta_param.h"
#include "cobotta_move/setting_data.h"
#include "ros/ros.h"


int main(int argc, char* argv[]){
    //ROS initialiaization
    ros::init(argc,argv,"cobotta_command_node");
    setting_data settings;
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
    getparam(settings,pnh);
    cobotta_move command(settings,&nh);
    ros::Rate loop_rate(settings.node_looprate);
    
    while(ros::ok()){
        if(command.update()==-1){
            //break;
        };
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}



   