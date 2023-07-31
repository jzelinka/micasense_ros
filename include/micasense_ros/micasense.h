#pragma once

#include "ros/ros.h"
#include "micasense_ros/micasense_params.h"
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

static const std::map<int, std::string> CHANNEL_NUM_TO_NAME = {
    {0, "blue"},
    {1, "green"},
    {2, "red"},
    {3, "red_edge"},
    {4, "near_infrared"},
    {5, "panchromatic"},
    {6, "thermal"},
};

class Micasense {
    public:
        Micasense(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    
    private:
        MicasenseParams params;
        static MicasenseParams load_params(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        std::string topic_name = "micasense/image";
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        image_transport::Publisher image_pub;
        std::stringstream response;

        bool show_timer = false;

        std::vector<image_transport::Publisher> image_pubs = std::vector<image_transport::Publisher>(7);
        std::vector<std::string> image_paths = std::vector<std::string>(7);
        std::vector<std::vector<char>> image_datas = std::vector<std::vector<char>>(7);

        bool camera_connected();
        bool camera_capture();
        bool parse_response();

        void publish_image(std::string image_path, unsigned int position);
        bool test_whole_process(std::string image_path);
        std::string pos_to_channel_name(unsigned int position);
        bool pos_valid(unsigned int position);

        // TODO check the calibration
};