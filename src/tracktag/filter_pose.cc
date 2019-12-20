//This is a file that subscribe pose of apriltag and 
//the do the filter to compensate and improve accuracy.
#include "filter.h"


#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>




int main(int argc , char** argv)
{
    ros::init(argc,argv,"pose_filter");
    ros::start(); // ros inilization 

    cv_filter apriltag_filter;

    ros::Rate loop_rate(100); 



    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe("rpicamerav2/apriltag/pose",10,&cv_filter::receivedata,&apriltag_filter);
    ros::Publisher pose_publisher = nh.advertise<nav_msgs::Odometry>("/uav2/px4_command/visualmeasurement",10);

    while (ros::ok())
    {
        if(apriltag_filter.ini == true){
            apriltag_filter.predict();
            
        }
        ros::spinOnce();
        apriltag_filter.publishpose(pose_publisher);
        
            
        loop_rate.sleep();
    }

}




