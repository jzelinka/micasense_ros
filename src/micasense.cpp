#include "micasense_ros/micasense.h"
#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <string>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Options.hpp>


Micasense::Micasense() {
    ROS_INFO("Micasense constructor.");
    std::cout << "Micasense constructor." << std::endl;
    int start = clock();

    std::ostringstream os;
    
    os << curlpp::options::Url(std::string("https://example.com/"));

    std::string asAskedInQuestion = os.str();
    std::cout << asAskedInQuestion << std::endl;
    std::cout << "Time is there: " << clock() - start << std::endl;
}