// Jun 9 created by Wang Renjie //

#include <string>
#include <vector>

#include "cam_cali/cam_pulish.hpp"

namespace calibration {

CameraNode::CameraNode(cv::VideoCapture cap){
    ros::NodeHandle camera_node;
    cap_ = cap;
    CameraNode::CamPublisher(camera_node);
}

void CameraNode::CamPublisher(ros::NodeHandle camera_node){
    image_transport::ImageTransport it(camera_node);
    image_transport::Publisher pub = it.advertise(img_topic.c_str(), 1);


    cv::Mat frame;
 
    ros::Rate loop_rate(30);
    while (camera_node.ok()) {
	    cap_.read(frame);
	    if(!frame.empty()){
			CameraNode::ImageProcess(frame);
	    	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	    	pub.publish(msg);
	    }
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void CameraNode::ImageProcess(cv::Mat frame){
	cv::flip(frame, frame, 0);
}

} // namespace calibration
