#include "ros/ros.h"
#include <thread>
#include "cam_cali/cam_node.hpp"

using namespace calibration;

void publish_cam(ros::NodeHandle camera_node, CameraNode cam){
    cam.CamPublisher(camera_node);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "img_publisher");
    ros::NodeHandle camera_node;
    CameraNode cam1;
    cam1.device_id = 0;
    cam1.img_topic = "camera1/image";
    std::thread cam1_thread(publish_cam, camera_node, cam1);
    cam1_thread.detach();

    CameraNode cam2;
    cam2.device_id = 2;
    cam2.img_topic = "camera2/image";
    cam2.CamPublisher(camera_node);
    return 0;
}