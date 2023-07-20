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

    for (size_t i = 0; i < this->image_pubs.size(); i++) {
        this->image_pubs[i] = it.advertise(this->topic_name + "_" + pos_to_channel_name(i), 1);
    }

    this->image_pub = it.advertise(this->topic_name, 1);

    // start periodical capture
    int start;

    // transport to machine
    ros::Rate loop_rate(0.5);
    while (ros::ok()) {
        ROS_INFO("cycle.");
        start = clock();
        camera_capture();
        std::cout << "capture: " << clock() - start << std::endl;
        start = clock();
        parse_response();
        std::cout << "parsing: " << clock() - start << std::endl;

        start = clock();
        for (size_t i = 0; i < this->image_paths.size(); i++) {
            publish_image(this->image_paths[i], i);
        }
        std::cout << "publishing: " << clock() - start << std::endl;

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

    std::string cache = json["raw_cache_path"].dump();

    // Alternatively, iterate using begin() and end() methods
    for (auto it = json["raw_cache_path"].begin(); it != json["raw_cache_path"].end(); ++it) {
        std::string key = it.key();
        auto& value = it.value();
        // TODO create some checks
        this->image_paths[std::stoi(key) - 1] = value;
    }

    return true;
}

bool Micasense::camera_capture() {
    // capture and fill the http response to be parsed using some json library
    curlpp::Cleanup cleanup;
    curlpp::Easy request;

    std::stringstream response;

    request.setOpt(curlpp::options::WriteStream(&response));

    request.setOpt(curlpp::options::Url(this->ip + "/capture?block=true&store_capture=false"));
    // request.setOpt(curlpp::options::Verbose(true));
    request.perform();

    this->response.str(std::string());
    this->response << response.str();

    return true;
}

bool Micasense::test_whole_process(std::string image_path) {
    // prepare the request
    curlpp::Cleanup cleanup;
    curlpp::Easy request;

    std::stringstream output;

    request.setOpt(curlpp::options::WriteStream(&output));

    request.setOpt(curlpp::options::Url(this->ip + image_path));
    // request.setOpt(curlpp::options::Verbose(true));

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


void Micasense::publish_image(std::string image_path, unsigned int position) {
    // prepare the request
    curlpp::Cleanup cleanup;
    curlpp::Easy request;

    std::stringstream output;

    request.setOpt(curlpp::options::WriteStream(&output));

    request.setOpt(curlpp::options::Url(this->ip + image_path));
    // request.setOpt(curlpp::options::Verbose(true));

    request.perform();

    output >> std::noskipws;
    std::vector<char> data;
    std::copy(std::istream_iterator<char>(output), std::istream_iterator<char>(), std::back_inserter(data));

    cv::Mat image = cv::imdecode(cv::Mat(data), cv::IMREAD_COLOR);

    if (!image.data) {
        ROS_WARN("Could not read image.");
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    if (pos_valid(position)) {
        this->image_pubs[position].publish(msg);
    } else {
        ROS_WARN("Invalid index of multispectral camera's channel.");
    }
}

std::string Micasense::pos_to_channel_name(unsigned int position) {
    if (!pos_valid(position)) {
        ROS_WARN("Invalid index of multispectral camera's channel.");
        return "";
    }

    auto it = CHANNEL_NUM_TO_NAME.find(position);
    if (it != CHANNEL_NUM_TO_NAME.end()) {
        return it->second;
    }     
}

bool Micasense::pos_valid(unsigned int position) {
    return position >= 0 && position < this->image_pubs.size();
}