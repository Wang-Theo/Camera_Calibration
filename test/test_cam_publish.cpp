#include "ros/ros.h"
#include "cam_cali/cam_node.hpp"

using namespace calibration;

int main(int argc, char **argv){
    ros::init(argc, argv, "img_publisher");
    ros::NodeHandle camera_node;
    CameraNode cam;
    cam.CamPublisher(camera_node);
    return 0;
}