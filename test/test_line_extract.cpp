#include "ros/ros.h"
#include "cam_cali/line_extract.hpp"

using namespace calibration;

int main(int argc, char **argv){
    ros::init(argc, argv, "line_extract_node");
    ros::NodeHandle line_extract_node;  
    LineExtract line_extract;
    line_extract.LineExtractSubscriber(line_extract_node);
    return 0;
}