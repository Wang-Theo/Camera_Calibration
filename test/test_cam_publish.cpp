#include "ros/ros.h"
#include "cam_cali/cam_node.hpp"

using namespace calibration;

int main(int argc, char **argv){
    ros::init(argc, argv, "img_publisher");

    cv::VideoCapture cap;
    int device_id=0;
    if(argc>1)
        device_id=argv[1][0]-'0';
    int api_id=cv::CAP_ANY;
    cap.open(device_id+api_id);
    if(!cap.isOpened()){
        std::cerr<<"ERROR! Unable to open camera"<<std::endl;
        return -1;
    }

    CameraNode cam(cap);
    return 0;
}