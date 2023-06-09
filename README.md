# Camera_Calibration

Tool to calibrate camera and publish images processed to RViz  

**Status: can publish images processed from camera**

## 1. Prerequisites
1.1 ROS neotic

1.2 OpenCV 4.2.0

1.3 C++ Standard 17

## 2. Execution
```
cd ~/catkin_ws/src
git clone https://github.com/Wang-Theo/Camera_Calibration.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES=cam_cali
source devel/setup.bash
roslaunch cam_cali cam_publish.launch 
```
