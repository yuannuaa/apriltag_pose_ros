#include <iostream>
#include <mutex>
#include <map>


#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


//Declaration
void GetPayloadPose(const nav_msgs::Odometry::ConstPtr& payloadmsg, const nav_msgs::Odometry::ConstPtr& uavmsg);
void FlagDetect(const std_msgs::String::ConstPtr& msg);
bool Loss_Flag = 1;
std::vector<std::pair<double , Eigen::Matrix4d>> relativeposedata;
Eigen::Vector3d CalculateVelocityFromPose();
ros::NodeHandle nh;
ros::Publisher pose_publisher = nh.advertise<nav_msgs::Odometry>("/uav2/px4_command/visualmeasurement",10);



//Definition
void GetPayloadPose(const nav_msgs::Odometry::ConstPtr& payloadmsg, const nav_msgs::Odometry::ConstPtr& uavmsg)
{
    nav_msgs::Odometry payload_msg = *payloadmsg;
    nav_msgs::Odometry uav_msg = *uavmsg;
    nav_msgs::Odometry pose_msg;
    double payload_time;
    

    //obtain payload state 
    Eigen::Quaternion<double> q_payload;
    q_payload.x() = payload_msg.pose.pose.orientation.x;
    q_payload.y() = payload_msg.pose.pose.orientation.y;
    q_payload.z() = payload_msg.pose.pose.orientation.z;
    q_payload.w() = payload_msg.pose.pose.orientation.w;
    Eigen::Matrix3d payload_R = q_payload.toRotationMatrix();
    Eigen::Vector3d payload_t;
    payload_t << payload_msg.pose.pose.position.x , payload_msg.pose.pose.position.y , payload_msg.pose.pose.position.z;
    Eigen::Matrix4d payload_T = Eigen::MatrixXd::Identity(4,4);
    payload_T.block<3,3>(0,0) = payload_R;
    payload_T.block<3,1>(0,3) = payload_t;

    //obtain UAV state
    Eigen::Quaternion<double> q_uav;
    q_uav.x() = uav_msg.pose.pose.orientation.x;
    q_uav.y() = uav_msg.pose.pose.orientation.y;
    q_uav.z() = uav_msg.pose.pose.orientation.z;
    q_uav.w() = uav_msg.pose.pose.orientation.w;
    Eigen::Matrix3d uav_R = q_uav.toRotationMatrix();
    Eigen::Vector3d uav_t;
    uav_t << uav_msg.pose.pose.position.x , uav_msg.pose.pose.position.y , uav_msg.pose.pose.position.z;
    Eigen::Matrix4d uav_T = Eigen::MatrixXd::Identity(4,4);
    uav_T.block<3,3>(0,0) = uav_R;
    uav_T.block<3,1>(0,3) = uav_t;

    //obtain relative pose
    payload_time = payload_msg.header.stamp.toSec();
    Eigen::Matrix4d relative_T;
    relative_T = uav_T.inverse() * payload_T;
    relativeposedata.push_back(std::make_pair(payload_time,relative_T));
    if (sizeof(relativeposedata) > 10){   
    Eigen::Vector3d relative_velocity = CalculateVelocityFromPose();
    
    //generate relative msg
    pose_msg.header = payload_msg.header;
    pose_msg.pose.pose.position.x = relative_T(3,0);
    pose_msg.pose.pose.position.x = relative_T(3,1);
    pose_msg.pose.pose.position.x = relative_T(3,2);
    pose_msg.pose.covariance[1] = 0;
    Eigen::Matrix3d relative_rotation = relative_T.block<3,3>(0,0);
    Eigen::Quaternion<double> q_relative(relative_rotation);
    pose_msg.pose.pose.orientation.x = q_relative.x();
    pose_msg.pose.pose.orientation.y = q_relative.y();
    pose_msg.pose.pose.orientation.z = q_relative.z();
    pose_msg.pose.pose.orientation.w = q_relative.w();
    pose_msg.twist.twist.linear.x =  relative_velocity(0);
    pose_msg.twist.twist.linear.y =  relative_velocity(1);
    pose_msg.twist.twist.linear.z =  relative_velocity(2);
    pose_publisher.publish(pose_msg);
    }

    



    if (Loss_Flag == 1){
        pose_msg.pose.covariance[1] = 1;
    }

}

void FlagDetect(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "on"){
        Loss_Flag = 0;
    }     
    else if (msg->data == "off"){
        Loss_Flag = 1;
    }
    else;
}


Eigen::Vector3d CalculateVelocityFromPose()
{
    Eigen::Vector3d payload_v_onestep;
    double dt = relativeposedata.back().first - relativeposedata[relativeposedata.size()-2].first;
    payload_v_onestep = (relativeposedata.back().second - relativeposedata[relativeposedata.size()-2].second).block<3,1>(0,3) / dt; 
    return payload_v_onestep;
    
}



//Main Function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "opencv_emulate");
    ros::start();

    


    message_filters::Subscriber<nav_msgs::Odometry> left_sub(nh, "/camera/left/image_raw", 10);
    message_filters::Subscriber<nav_msgs::Odometry> right_sub(nh, "/camera/right/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&GetPayloadPose,_1,_2));
}

