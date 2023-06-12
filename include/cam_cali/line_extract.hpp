#ifndef LINE_EXTRACT
#define LINE_EXTRACT

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "opencv2/imgproc.hpp"  
#include "opencv2/ximgproc.hpp"  
#include "opencv2/imgcodecs.hpp"  
#include "opencv2/highgui.hpp"  

#include <chrono>
#include <iostream>
#include <boost/bind.hpp>

namespace calibration {

class LineExtract{
    public:
        struct sort_descriptor_by_queryIdx
        {
            inline bool operator()(const std::vector<cv::DMatch>& a, const std::vector<cv::DMatch>& b){
                return ( a[0].queryIdx < b[0].queryIdx );
            }
        };

        struct sort_lines_by_response
        {
            inline bool operator()(const cv::line_descriptor::KeyLine& a, const cv::line_descriptor::KeyLine& b){
                return ( a.response > b.response );
            }
        };

    public:
        std::string img1_topic = "camera1/image";
        std::string img2_topic = "camera2/image";

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> SyncPolicy;
        message_filters::Subscriber<sensor_msgs::Image>* subscriber_image1 ;             // topic1 输入
        message_filters::Subscriber<sensor_msgs::Image>* subscriber_image2;   // topic2 输入
        message_filters::Synchronizer<SyncPolicy>* sync_;

        ros::Publisher pub_;

    public:
        void LineExtractSubscriber(ros::NodeHandle nh);
        void ImageCallback(const sensor_msgs::ImageConstPtr& msg_cam1,  const sensor_msgs::ImageConstPtr& msg_cam2);

        cv::Mat FastLineDetector(cv::Mat image1, cv::Mat image2);
        cv::Mat ExtractLineSegment(const cv::Mat &image1, const cv::Mat &image2, 
                                    std::vector<cv::line_descriptor::KeyLine> &keylines1,
                                    std::vector<cv::line_descriptor::KeyLine> &keylines2);
};


} // namespace calibration

#endif /* LINE_EXTRACT */
