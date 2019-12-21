# apriltag_pose_ros

This is a ROS wrapper for publishing Apriltag with Jetson nano and RPI camera through CSI

It is truely easy to use. On jetson nano with ROS. 

catkin_make

and everything is OK

Dependency

AprilTag  https://github.com/AprilRobotics/apriltag 

Follow the instruction to do

git clone https://github.com/AprilRobotics/apriltag 

cmake .

sudo make install

21 Dec 2019
To avoid some out of view situation , a constant velocity kalman filter has been added to the package. It now can output 100Hz pose estimation of apriltag and able to compensate some undesirable situation.
