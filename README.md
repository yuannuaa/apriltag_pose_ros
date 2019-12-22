# apriltag_pose_ros
This is a ROS wrapper for publishing Apriltag with Jetson nano and RPI camera V2 through CSI  
It is truely easy to use. On jetson nano with ROS. A constant velocity EKF filter is introducted to enhance the performance.  
## Complie Instruciton
```
catkin_make
```

### Dependency

 - Ubuntu 16.04 + ROS Kinetic
 - AprilTag
    > [AprilTag](https://github.com/AprilRobotics/apriltag)  
    > Follow the instruction to do  
    > ```
    > git clone https://github.com/AprilRobotics/apriltag  
    > cmake .  
    > sudo make install  
    >  ```
 - ROS
	Please follow the official instruction to install ros-kinetic 
	>[http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)    
- Eigen3  
    >	```
	>	sudo apt install libeigen3-dev
	>	``` 
- OpenCV
	> ```
	> sudo apt install libopencv-dev
	> ```

## Function Description 

21 Dec 2019

To avoid some out of view situation , a constant velocity kalman filter has been added to the package. It now can output 100Hz pose estimation of apriltag and able to compensate some undesirable situation.