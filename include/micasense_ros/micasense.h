#pragma once

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Infos.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <nlohmann/json.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class Micasense {
    public:
        Micasense(ros::NodeHandle& nh);
    
    private:
        // TODO read the ip from the parameter server
        std::string ip = "http://192.168.1.83";
        std::string topic_name = "micasense/image";
        ros::NodeHandle nh;
        image_transport::Publisher image_pub;
        std::stringstream response;

        std::vector<std::string> image_paths = std::vector<std::string>(7);

        bool camera_connected();
        bool camera_capture();
        bool parse_response();

        void publish_image(std::string image_path);
        bool test_whole_process(std::string image_path);
    
    // check if the camera is connected
    // get parameters as the wanted ip
    // if connected then start receiving images
    // stream the images to the ros topic
};