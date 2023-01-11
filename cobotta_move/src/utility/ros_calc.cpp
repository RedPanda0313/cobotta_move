#include "utility/ros_calc.h"

geometry_msgs::Pose utility::multiply_orientation(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
    geometry_msgs::Pose temp;
    temp.orientation.w = p1.orientation.w * p2.orientation.w - p1.orientation.x * p2.orientation.x - p1.orientation.y * p2.orientation.y - p1.orientation.z * p2.orientation.z;
    temp.orientation.x = p1.orientation.w * p2.orientation.x + p1.orientation.x * p2.orientation.w + p1.orientation.y * p2.orientation.z - p1.orientation.z * p2.orientation.y;
    temp.orientation.y = p1.orientation.w * p2.orientation.y - p1.orientation.x * p2.orientation.z + p1.orientation.y * p2.orientation.w + p1.orientation.z * p2.orientation.x;
    temp.orientation.z = p1.orientation.w * p2.orientation.z + p1.orientation.x * p2.orientation.y - p1.orientation.y * p2.orientation.x + p1.orientation.z * p2.orientation.w;
    return temp;
}

geometry_msgs::Pose utility::inverse_orientation(geometry_msgs::Pose p1){
    geometry_msgs::Pose temp;
    temp.orientation.w = p1.orientation.w;
    temp.orientation.x = -p1.orientation.x;
    temp.orientation.y = -p1.orientation.y;
    temp.orientation.z = -p1.orientation.z;
    return temp;
}