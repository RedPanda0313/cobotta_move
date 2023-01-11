#pragma once
#include "geometry_msgs/Pose.h"

namespace utility{
    geometry_msgs::Pose multiply_orientation(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
    geometry_msgs::Pose inverse_orientation(geometry_msgs::Pose p1);
    
}
