# Camera_Calibration

![Ubuntu 20.04](https://img.shields.io/badge/OS-Ubuntu_20.04-informational?style=flat&logo=ubuntu&logoColor=white&color=2bbc8a)
![ROS Noetic](https://img.shields.io/badge/Tools-ROS_Noetic-informational?style=flat&logo=ROS&logoColor=white&color=2bbc8a)
![C++](https://img.shields.io/badge/Code-C++-informational?style=flat&logo=c%2B%2B&logoColor=white&color=2bbc8a)

Tool to calibrate multi-cameras and camera-lidar

**Status: publish camera images / calibrate camera intrinsics / extract line feature**

## 1. Prerequisites
1.1 ROS neotic

1.2 OpenCV 4.2.0

1.3 C++ Standard 17

## 2. Execution
2.1 Build
```
cd ~/catkin_ws/src
git clone https://github.com/Wang-Theo/Camera_Calibration.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES=cam_cali
```

2.2 Publish two cameras' images
```
source devel/setup.bash
roslaunch cam_cali cam_publish.launch 
```
<img src="image/cam_publish_.png" width="500" alt="image_publish"/>

2.3 Calibrate camera intrinsics
```
source devel/setup.bash
rosrun cam_cali cam_calibrate
```
You can find the calibrated image `calibrated_image.png` in folder `image`  
<img src="image/calibrated_image.png" width="500" alt="image_calibrated"/>

The calbrated result matrix is shown in terminal  
<img src="image/cam_cali_result.png" width="500" alt="calibrate_result"/>

2.4 Extract line feature
```
source devel/setup.bash
rosrun cam_cali line_extract
```
<img src="image/line_extract_example.png" width="500" alt="line_extracted"/>
