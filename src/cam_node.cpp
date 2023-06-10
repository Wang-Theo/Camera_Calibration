// Jun 9 created by Wang Renjie //

#include <string>
#include <vector>

#include "cam_cali/cam_node.hpp"

namespace calibration {

CameraNode::CameraNode(cv::VideoCapture cap){
    ros::NodeHandle camera_node;
    cap_ = cap;
    CameraNode::CamPublisher(camera_node);
}

void CameraNode::CamPublisher(ros::NodeHandle camera_node){
    image_transport::ImageTransport it(camera_node);
    image_transport::Publisher pub = it.advertise(img_topic.c_str(), 1);
 
    ros::Rate loop_rate(30);
    int img_id = 1;
    int image_exist_flag = 0;
    while (camera_node.ok()) {
	    cap_.read(frame);
	    if(!frame.empty()){
            filename = "/home/renjie/catkin_ws/src/Camera_Calibration/image/img" + std::to_string(img_id) + ".png"; // modify filepath of images to be saved
            cv::imwrite(filename.c_str(), frame);

            if (image_exist_flag == 1){
                CameraNode::OpencvCalibrateCameraIntrinsics(); // process images from camera
            }

	    	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	    	pub.publish(msg);
	    }
        if(img_id == img_num){
            img_id = 1;
            image_exist_flag = 1;
        }
        img_id++;
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void CameraNode::OpencvCalibrateCameraIntrinsics(){
    // 1. Define checkerboard parameters
    int boardWidth = 6;  // horizontal corner number
    int boardHeight = 8; // vertical corner number
    float squareSize = 1.f; // grid size (unit: meter)(can set roughly)
    cv::Size boardSize(boardWidth, boardHeight);

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Point2f> corners;


    // 2. Extracte corner points
    cv::Mat image, gray;
    bool found;
    for (size_t i = 0; i < img_num; i++)
    {
        image = cv::imread((filename.substr(0,filename.length()-5)+std::to_string(i+1)+filename.substr(filename.length()-4,filename.length())).c_str(), cv::IMREAD_COLOR);
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        found = cv::findChessboardCorners(image, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        if (found)
        {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            cv::drawChessboardCorners(image, boardSize, corners, found);

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
    if(found){
        cv::Mat cameraMatrix, distCoeffs;
        std::vector<cv::Mat> rvecs, tvecs;
        cv::calibrateCamera(objectPoints, imagePoints, image.size(), cameraMatrix, distCoeffs, rvecs, tvecs);
        frame = image;
        cv::imwrite("/home/renjie/catkin_ws/src/Camera_Calibration/image/calibrated_image.png",image);

        std::cout << "Camera matrix:" << std::endl << cameraMatrix << std::endl;
        std::cout << "Distortion coefficients:" << std::endl << distCoeffs << std::endl;
        std::cout << "----------------------------------" << std::endl;
        
    }
    std::cout << "Checkerboard not found or too ambiguous !" << std::endl;
}

} // namespace calibration
