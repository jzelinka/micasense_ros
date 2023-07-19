#include "micasense_ros/micasense.h"

Micasense::Micasense(ros::NodeHandle& nh) {
    ROS_INFO("Micasense constructor.");

    // test if camera connected
    if (!camera_connected()) {
        ROS_INFO("Shutting down node, camera not reachable on %s.", this->ip.c_str());
        ros::shutdown();
    }

    // prepare publisher
    this->nh = nh;
    this->image_pub = this->nh.advertise<sensor_msgs::Image>("micasense/image", 1);

    // start periodical capture

    // transport to machine

    // publish to ros topic
}

bool Micasense::camera_connected() {
    if (this->ip == "") {
        ROS_INFO("No IP address provided.");
        return false;
    }

    try {
        curlpp::Cleanup cleanup;
        curlpp::Easy request;
        request.setOpt(curlpp::options::Url(this->ip));

        request.setOpt(curlpp::options::Timeout(1));
        std::ostringstream nullStream;
        request.setOpt(curlpp::options::WriteStream(&nullStream));

        request.perform();
        long response_code;
        curlpp::InfoGetter::get(request, CURLINFO_RESPONSE_CODE, response_code);
        if (response_code == 200) {
            ROS_INFO("Camera connected.");
            return true;
        }
    }  catch (curlpp::RuntimeError &e) {
        return false;
    } catch (curlpp::LogicError &e) {
        return false;
    }

    return true;
}
