#include "ros/ros.h"
#include "micasense_ros/micasense.h"

int main(int argc, char **argv)
{
    ROS_INFO("Starting Micasense camera.");

    ros::init(argc, argv, "micasense");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); 
    Micasense micasense(nh, pnh);
    return 0;
}