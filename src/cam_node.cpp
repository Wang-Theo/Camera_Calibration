// Jun 9 created by Wang Renjie //

#include "cam_cali/cam_node.hpp"

namespace calibration {

void CameraNode::OpencvCalibrateCameraIntrinsics(std::queue<cv::Mat> images){
    // 1. Define checkerboard parameters
    int boardWidth = 6;  // horizontal corner number
    int boardHeight = 8; // vertical corner number
    float squareSize = 1.f; // grid size (unit: meter)(can set roughly)
    cv::Size boardSize(boardWidth, boardHeight);

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Point2f> corners;

    // 2. Extracte corner points
    cv::Mat image_, gray;
    bool found;
    for (size_t i = 0; i < img_num; i++)
    {   
        image_ = images.front().clone();
        images.pop();
        cv::cvtColor(image_, gray, cv::COLOR_BGR2GRAY);

        found = cv::findChessboardCorners(image_, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        if (found)
        {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            cv::drawChessboardCorners(image_, boardSize, corners, found);

            std::vector<cv::Point3f> objectCorners;
            for (int j = 0; j < boardHeight; j++)
            {
                for (int k = 0; k < boardWidth; k++)
                {
                    objectCorners.push_back(cv::Point3f(k * squareSize, j * squareSize, 0));
                }
            }
            objectPoints.push_back(objectCorners);
            imagePoints.push_back(corners);
        }
    }

    // 3. Calibrate camera
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    if(imagePoints.size()!=0){
        cv::calibrateCamera(objectPoints, imagePoints, image_.size(), cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K3);

        cv::imwrite("/home/renjie/catkin_ws/src/Camera_Calibration/image/calibrated_image.png",image_);

        std::cout << "Camera matrix:" << std::endl << cameraMatrix << std::endl;
        std::cout << "Distortion coefficients:" << std::endl << distCoeffs << std::endl;
        std::cout << "----------------------------------" << std::endl;
    }else{
        std::cout << "Checkerboard not found or too ambiguous !" << std::endl;
    }
}

void CameraNode::ImageCallback(const sensor_msgs::ImageConstPtr& msg){
    std::cout<<"Start calibration ! (image number: "<< img_num << ")" <<std::endl;
    cv::Mat images_subcribe = cv_bridge::toCvShare(msg, "rgb8")->image;
    int img_id = 1;
    bool flag = false;
    while(!images_subcribe.empty()){
        img_saved.push(images_subcribe);
        if (flag == true) {
            img_saved.pop();
            CameraNode::OpencvCalibrateCameraIntrinsics(img_saved);
        }

        if(img_id == img_num){
            image_exist_flag = 1;
            flag = true;
        }
        img_id++;
    }
}

void CameraNode::CamPublisher(ros::NodeHandle nh){
    cv::VideoCapture cap;
    int api_id=cv::CAP_ANY;
    cap.open(device_id+api_id);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(img_topic.c_str(), 1);
 
    ros::Rate loop_rate(30);
    while (nh.ok()) {
	    cap.read(frame);
	    if(!frame.empty()){
	    	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	    	pub.publish(msg);
	    }
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void CameraNode::CamSubscriber(ros::NodeHandle nh){
    image_transport::ImageTransport it(nh); 
    image_transport::Subscriber sub = it.subscribe(img_topic.c_str(), 1, &CameraNode::ImageCallback, this);  
    ros::spin();  
}

} // namespace calibration
