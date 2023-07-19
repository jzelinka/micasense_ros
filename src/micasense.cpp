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
        parse_response();

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

bool Micasense::parse_response() {
    nlohmann::json json = nlohmann::json::parse(this->response.str());

    std::cout << this->response.str() << std::endl;

    std::string cache = json["raw_cache_path"].dump();

    // Alternatively, iterate using begin() and end() methods
    for (auto it = json["raw_cache_path"].begin(); it != json["raw_cache_path"].end(); ++it) {
        std::string key = it.key();
        auto& value = it.value();
        std::cout << "Key: " << key << ", Value: " << value << std::endl;
    }

    std::cout << cache << std::endl;
    return true;
}

bool Micasense::camera_capture() {
    // capture and fill the http response to be parsed using some json library
    curlpp::Cleanup cleanup;
    curlpp::Easy request;

    std::stringstream response;

    request.setOpt(curlpp::options::WriteStream(&response));

    request.setOpt(curlpp::options::Url(this->ip + "/capture?block=true&store_capture=false"));
    request.setOpt(curlpp::options::Verbose(true));
    request.perform();

    this->response.str(std::string());
    this->response << response.str();

    return true;
}

bool Micasense::test_whole_process() {
    // prepare the request
    curlpp::Cleanup cleanup;
    curlpp::Easy request;


    std::stringstream output;

    request.setOpt(curlpp::options::WriteStream(&output));

    request.setOpt(curlpp::options::Url("http://192.168.1.83/images/tmp0.tif"));
    request.setOpt(curlpp::options::Verbose(true));

    request.perform();

    output >> std::noskipws;
    std::vector<char> data;
    std::copy(std::istream_iterator<char>(output), std::istream_iterator<char>(), std::back_inserter(data));

    cv::Mat image = cv::imdecode(cv::Mat(data), cv::IMREAD_COLOR);

    if (!image.data) {
        ROS_INFO("Could not read image.");
        return false;
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    this->image_pub.publish(msg);
    
    // send the request to the camera
    // process the output
    return true;
}


void Micasense::publish_image(std::string image_path) {
    // TODO make publish_path on robot and topic
    ROS_INFO("Publishing image. From %s", image_path.c_str());
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

    if (!image.data) {
        ROS_INFO("Could not read image.");
        return;
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    this->image_pub.publish(msg);
}