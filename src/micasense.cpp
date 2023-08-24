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
    pnh.getParam("save_on_camera_card", params.save_on_camera_card);
    pnh.getParam("overwrite_timestamp", params.overwrite_timestamp);

    // setting the bitmask
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

void Micasense::send_timestamp_for_capture() {
    curlpp::Cleanup cleanup;
    curlpp::Easy request;

    // set the time
    std::string url = this->params.get_ip() + "/capture_state";

    std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();

    // Convert the time point to time_t
    std::time_t currentTime_t = std::chrono::system_clock::to_time_t(currentTime);
    this->capture_time = ros::Time::now();

    // Convert time_t to a string
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000000Z", std::localtime(&currentTime_t));
    nlohmann::json data;
    data["utc_time"] = buffer;
    
    // need to wait for the camera to be ready
    ros::Duration(0.4).sleep();
    std::list<std::string> header;
    header.push_back("Content-Type: application/json");

    request.setOpt(curlpp::options::Url(url));
    request.setOpt(curlpp::options::HttpHeader(header));
    request.setOpt(curlpp::options::PostFields(data.dump()));
    request.setOpt(new curlpp::options::PostFieldSize(data.dump().length()));

    response.str(std::string());
    request.setOpt(curlpp::options::WriteStream(&response));
    request.perform();
}

bool Micasense::camera_capture() {
    // capture and fill the http response to be parsed using some json library
    curlpp::Cleanup cleanup;
    curlpp::Easy request;

    std::stringstream response;

    auto start = std::chrono::high_resolution_clock::now();

    request.setOpt(curlpp::options::WriteStream(&response));

    std::string store_capture_val = this->params.save_on_camera_card ? "true" : "false";
    std::string url = this->params.get_ip() + "/capture?use_post_capture_state=true&store_capture=" + store_capture_val + "&cache_raw=" + std::to_string(this->params.get_channel_bit_mask());
    std::cout << "url: " << url << std::endl;

    request.setOpt(curlpp::options::Url(url));
    request.perform();

    // get capture id
    nlohmann::json json = nlohmann::json::parse(response.str());
    std::string capture_id = json["id"].dump();

    if (this->params.overwrite_timestamp) {
        this->send_timestamp_for_capture();
    }

    url = this->params.get_ip() + "/capture/" + capture_id.substr(1, capture_id.size() - 2);
    bool capture_complete = false;
    curlpp::Easy capture_request;
    capture_request.setOpt(new curlpp::options::Url(url));

    while (!capture_complete) {

        response.str(std::string());
        capture_request.setOpt(new curlpp::options::WriteStream(&response));
        capture_request.perform();

        nlohmann::json capture_status_json  = nlohmann::json::parse(response.str());

        capture_complete = (capture_status_json["status"] == "complete");

        if (!capture_complete) {
            ros::Duration(0.1).sleep();
        }
    }

    this->set_response(&response);

    if (this->show_timer) {
        auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start);
        std::cout << "capture and parse json: " << ms_int.count() << std::endl;
    }

    return true;
}

void Micasense::set_response(std::stringstream* response) {
    this->response.str(std::string());
    this->response << response->str();
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
    // using std::chrono::high_resolution_clock;
    // using std::chrono::duration_cast;
    // using std::chrono::duration;
    // using std::chrono::milliseconds;

    // auto t1 = high_resolution_clock::now();
    // long_operation();
    // auto t2 = high_resolution_clock::now();

    // /* Getting number of milliseconds as an integer. */
    // auto ms_int = duration_cast<milliseconds>(t2 - t1);

    // /* Getting number of milliseconds as a double. */
    // duration<double, std::milli> ms_double = t2 - t1;

    // std::cout << ms_int.count() << "ms\n";
    // std::cout << ms_double.count() << "ms\n";
    // return 0;

void Micasense::publish_image(std::string image_path, unsigned int position) {
    curlpp::Cleanup cleanup;
    curlpp::Easy request;

    if (image_path == "") {
        return;
    }

    std::string tmp_response_file = "/tmp/response.tif";

    auto start = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<std::chrono::milliseconds> ms_int;
    // if (this->show_timer) start = std::chrono::high_resolution_clock::now();
    std::ofstream outputFile(tmp_response_file, std::ios::binary);

    // Set the output stream to the file stream
    request.setOpt(curlpp::options::WriteStream(&outputFile));

    request.setOpt(curlpp::options::Url(this->params.get_ip() + image_path));

    request.perform();

    outputFile.close();
    if (this->show_timer) {
        auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start);
        std::cout << "downloading: " << ms_int.count() << std::endl;
    }

    if (this->show_timer) start = std::chrono::high_resolution_clock::now();


    cv::Mat image = cv::imread(tmp_response_file, cv::IMREAD_COLOR);
    if (this->show_timer) {
        // std::cout << "imdecode: " << std::chrono::high_resolution_clock::now() - start << std::endl;
        auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start);
        std::cout << "imdecode: " << ms_int.count() << std::endl;
    }
    if (this->show_timer) start = std::chrono::high_resolution_clock::now();

    if (!image.data) {
        ROS_WARN("Could not read image.");
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    msg->header.stamp = this->capture_time;
    msg->encoding = "mono8";

    if (pos_valid(position)) {
        this->image_pubs[position].publish(msg);
    } else {
        ROS_WARN("Invalid index of multispectral camera's channel.");
    }
    if (this->show_timer) {
        // std::cout << "publishing: " << 
        // std::chrono::high_resolution_clock::now() - start << std::endl;
        auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start);
        std::cout << "publish: " << ms_int.count() << std::endl;
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
    return "";
}

bool Micasense::pos_valid(unsigned int position) {
    return position >= 0 && position < this->image_pubs.size();
}