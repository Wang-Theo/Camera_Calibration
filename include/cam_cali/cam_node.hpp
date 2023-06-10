#ifndef CAM_SUBSCRIBE
#define CAM_SUBSCRIBE

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect.hpp>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/covariance.h"

#include <iostream>

namespace calibration {
class CameraNode{
    public:
        CameraNode(cv::VideoCapture cap);

        void CamPublisher(ros::NodeHandle camera_node);
        void ImageProcess(cv::Mat frame);

        ros::NodeHandle camera_node;
        cv::VideoCapture cap_;
        std::string img_topic="camera/image";
        int img_num = 9;
        std::string filename;
};
} // namespace calibration

#endif /* CAM_SUBSCRIBE */
