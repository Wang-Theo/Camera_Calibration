#include "ros/ros.h"
#include "cam_cali/cam_node.hpp"

using namespace calibration;

int main(int argc, char **argv){
    ros::init(argc, argv, "cam_sub_node");
    ros::NodeHandle camera_node;  
    CameraNode cam;
    cam.img_topic = "camera1/image";
    cam.CamSubscriber(camera_node);
    return 0;
}