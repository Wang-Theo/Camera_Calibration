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
#include <vector>
#include <string>
#include <queue>
#include <thread>

namespace calibration {
class CameraNode{
    public:
        int device_id=2;
        std::string img_topic="camera/image";
        cv::Mat frame;

        int img_num = 20;
        std::queue<cv::Mat> img_saved;
        int image_exist_flag = 0;

    public:
        void OpencvCalibrateCameraIntrinsics(std::queue<cv::Mat> images);
        void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

        void CamPublisher(ros::NodeHandle nh);
        void CamSubscriber(ros::NodeHandle nh);
};
} // namespace calibration

#endif /* CAM_SUBSCRIBE */
