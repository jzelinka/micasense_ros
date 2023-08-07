#include "micasense_ros/micasense.h"

Micasense::Micasense(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    ROS_INFO("Micasense constructor.");

    this->nh = nh;
    this->pnh = pnh;

    this->params = load_params(nh, pnh);

    if (!camera_connected()) {
        ROS_INFO("Shutting down node, camera not reachable on %s.", this->params.get_ip().c_str());
        ros::shutdown();
    }

    image_transport::ImageTransport it(this->nh);

    for (size_t i = 0; i < this->image_pubs.size(); i++) {
        if (this->params.get_channel_bit_mask() & (1 << i)) {
            this->image_pubs[i] = it.advertise(this->topic_name + "_" + pos_to_channel_name(i), 1);
        }
    }

    // this->image_pub = it.advertise(this->topic_name, 1);

    ros::Rate loop_rate(2.0);
    while (ros::ok()) {
        ROS_INFO("Capturing.");
        camera_capture();
        parse_response();

        for (size_t i = 0; i < this->image_paths.size(); i++) {
            publish_image(this->image_paths[i], i);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

MicasenseParams Micasense::load_params(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    MicasenseParams params;

    ROS_INFO("loading params");
    if (!pnh.getParam("ip", params.ip)) {
        params.ip = "192.168.1.83";
    }

    // get all the params
    pnh.getParam("use_red", params.use_red);
    pnh.getParam("use_blue", params.use_blue);
    pnh.getParam("use_green", params.use_green);
    pnh.getParam("use_red_edge", params.use_red_edge);
    pnh.getParam("use_near_infrared", params.use_near_infrared);
    pnh.getParam("use_panchromatic", params.use_panchromatic);
    pnh.getParam("use_thermal", params.use_thermal);

    int tmp_bit_mask = 0;
    if (params.use_red) tmp_bit_mask |= 1 << 2;
    if (params.use_blue) tmp_bit_mask |= 1 << 0;
    if (params.use_green) tmp_bit_mask |= 1 << 1;
    if (params.use_red_edge) tmp_bit_mask |= 1 << 3;
    if (params.use_near_infrared) tmp_bit_mask |= 1 << 4;
    if (params.use_panchromatic) tmp_bit_mask |= 1 << 5;
    if (params.use_thermal) tmp_bit_mask |= 1 << 6;

    params.set_channel_bit_mask(tmp_bit_mask);

    std::cout << "ip: " << params.get_ip() << std::endl;
    std::cout << "bit_mask: " << params.get_channel_bit_mask() << std::endl;

    params.assert_valid();
    return params;
}

bool Micasense::camera_connected() {
    if (this->params.get_ip() == "") {
        ROS_INFO("No IP address provided.");
        return false;
    }

    try {
        curlpp::Cleanup cleanup;
        curlpp::Easy request;
        request.setOpt(curlpp::options::Url(this->params.get_ip()));

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
        if (std::stoi(key) - 1 >= this->image_paths.size()) {
            ROS_WARN("Invalid index of multispectral camera's channel.");
            return false;
        }
        
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

    std::string url = this->params.get_ip() + "/capture?block=true&store_capture=false&cache_raw=" + std::to_string(this->params.get_channel_bit_mask());
    std::cout << "url: " << url << std::endl;

    request.setOpt(curlpp::options::Url(url));
    request.perform();

    this->response.str(std::string());
    this->response << response.str();

    return true;
}

// bool Micasense::test_whole_process(std::string image_path) {
//     // prepare the request
//     curlpp::Cleanup cleanup;
//     curlpp::Easy request;

//     std::stringstream output;

//     request.setOpt(curlpp::options::WriteStream(&output));

//     request.setOpt(curlpp::options::Url(this->params.get_ip() + image_path));

//     request.perform();

//     output >> std::noskipws;
//     std::vector<char> data;
//     std::copy(std::istream_iterator<char>(output), std::istream_iterator<char>(), std::back_inserter(data));

//     cv::Mat image = cv::imdecode(cv::Mat(data), cv::IMREAD_COLOR);

//     if (!image.data) {
//         ROS_INFO("Could not read image.");
//         return false;
//     }

//     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

//     this->image_pub.publish(msg);

//     return true;
// }


void Micasense::publish_image(std::string image_path, unsigned int position) {
    curlpp::Cleanup cleanup;
    curlpp::Easy request;

    if (image_path == "") {
        return;
    }

    std::string tmp_response_file = "/tmp/response.tif";

    int start;
    if (this->show_timer) start = clock();
    std::ofstream outputFile(tmp_response_file, std::ios::binary);

    // Set the output stream to the file stream
    request.setOpt(curlpp::options::WriteStream(&outputFile));

    request.setOpt(curlpp::options::Url(this->params.get_ip() + image_path));

    request.perform();

    outputFile.close();
    if (this->show_timer) std::cout << "downloading: " << clock() - start << std::endl;

    if (this->show_timer) start = clock();


    cv::Mat image = cv::imread(tmp_response_file, cv::IMREAD_COLOR);
    if (this->show_timer) std::cout << "imdecode: " << clock() - start << std::endl;
    if (this->show_timer) start = clock();

    if (!image.data) {
        ROS_WARN("Could not read image.");
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    if (pos_valid(position)) {
        this->image_pubs[position].publish(msg);
    } else {
        ROS_WARN("Invalid index of multispectral camera's channel.");
    }
    if (this->show_timer) std::cout << "publishing: " << clock() - start << std::endl;
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
    return "";
}

bool Micasense::pos_valid(unsigned int position) {
    return position >= 0 && position < this->image_pubs.size();
}