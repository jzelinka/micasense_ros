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
    image_transport::ImageTransport it(this->nh);
    this->image_pub = it.advertise(this->topic_name, 1);

    // start periodical capture

    // transport to machine
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        camera_capture();
        // publish_image("/home/jz/multi_spec/simple_collect/1.tif");
        ros::spinOnce();
        loop_rate.sleep();
    }

    // publish to ros topic

    // publish image from file - start publishing tiff images
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

bool Micasense::camera_capture() {
    // prepare the request
    curlpp::Cleanup cleanup;
    curlpp::Easy request;
    request.setOpt(curlpp::options::Url(this->ip + "/capture?cache_raw=31"));
    request.perform();

    request.setOpt(curlpp::options::Url(this->ip + "/images/tmp0.tif"));
    request.perform();
    
    // send the request to the camera
    // process the output
    return true;
}


void Micasense::publish_image(std::string image_path) {
    ROS_INFO("Publishing image. From %s", image_path.c_str());
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

    if (!image.data) {
        ROS_INFO("Could not read image.");
        return;
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    this->image_pub.publish(msg);
}