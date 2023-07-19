#pragma once

#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <string>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Infos.hpp>
#include <sensor_msgs/Image.h>


class Micasense {
    public:
        Micasense(ros::NodeHandle& nh);
    
    private:
        // TODO read the ip from the parameter server
        std::string ip = "http://192.168.1.83";
        ros::NodeHandle nh;
        ros::Publisher image_pub;
        bool camera_connected();
        bool camera_capture();
    
    // check if the camera is connected
    // get parameters as the wanted ip
    // if connected then start receiving images
    // stream the images to the ros topic
};