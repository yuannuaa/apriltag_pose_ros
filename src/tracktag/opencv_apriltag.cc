/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <iostream>
#include <math.h>
#include "opencv2/opencv.hpp"   


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

#include "unistd.h"
#include <chrono>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "common/getopt.h"
#include "apriltag_pose.h"
}


using namespace std;
using namespace cv;


std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}



int main(int argc, char *argv[])
{

    ros::init(argc, argv, "track_april_tag");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("rpicamerav2/image_raw", 1);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("rpicamerav2/apriltag/pose", 1);
    sensor_msgs::ImagePtr msg;
   
    ros::Rate loop_rate(30);

    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2am.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize camera
    // undistort
 /*       FileStorage fs("/home/jetson1/Apriltag_pose_ros/src/tracktag/parameter.yaml",cv::FileStorage::READ);
        if(!fs.isOpened()) {
                cerr<<"Failed to open CamMatrix"<<endl;
                system("pause");
                exit(EXIT_FAILURE);
        }
        cv::Mat matrix;
        cv::Mat coeff;
        cv::Mat new_matrix;//matrix after undistort
        fs["camera_matrix"]>>matrix;
                fs["distortion_coefficients"]>>coeff;
                cout<<"Matrix"<<matrix<<std::endl;
                cout<<"Coeff"<<coeff<<std::endl;
                cout<<" load parameters success! "<<endl;*/
// Initialize Camera through CSI
    int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 1280 ;
    int display_height = 720 ;
    int framerate = 30 ;
    int flip_method = 0 ;

    std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
// normal initialize
  //  cv::VideoCapture cap(1);
    if(!cap.isOpened()) {
	std::cout<<"Failed to open camera."<<std::endl;
	return (-1);
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }




    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

// initialize apriltag pose estimation parameter
    apriltag_detection_info_t info;
    
    info.tagsize = 0.0578;
    info.fx = 1272.31172;
    info.fy = 1273.235019;
    info.cx = 648.622369;
    info.cy = 362.708529;


    Mat frame, gray , un_gray;

    while (ros::ok()) {
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
     //   undistort(gray,un_gray,matrix,coeff,new_matrix);
        ros::Time time_c= ros::Time::now (); 
        if (!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();  
	        msg->header.stamp = time_c;  
            image_pub.publish(msg);  
        }

//ros camera publish



        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);
        cout << zarray_size(detections) << " tags detected" << endl;
        geometry_msgs::PoseStamped pose_msg0;
        pose_msg0.header.stamp = time_c;
        pose_msg0.pose.position.x = 0;
        pose_msg0.pose.position.y = 0;
        pose_msg0.pose.position.z = 0;


      //  double * quater = Rotation_Quaternion (r11,r12,r13,r21,r22,r23,r31,r32,r33);
        pose_msg0.pose.orientation.w = time_c.toSec();
        pose_msg0.pose.orientation.x = 0;
        pose_msg0.pose.orientation.y = 0;
        pose_msg0.pose.orientation.z = 0;
        pose_pub.publish(pose_msg0);


        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

	    	// Then call estimate_tag_pose.
	    info.det = det;
            apriltag_pose_t pose;
	    double err = estimate_tag_pose(&info, &pose);
	    // Do something with pose.
	    std::cout << pose.t->data[0]<<"\t"<<pose.t->data[1]<<"\t"<<pose.t->data[2]<<std::endl;
	   
	    //publish pose 


            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = time_c;
            pose_msg.pose.position.x = pose.t->data[0];
            pose_msg.pose.position.y = pose.t->data[1];
            pose_msg.pose.position.z = pose.t->data[2];
	        double r11 = pose.R->data[0];   
	        double r12 = pose.R->data[1];   
	        double r13 = pose.R->data[2];   
	        double r21 = pose.R->data[3];   
	        double r22 = pose.R->data[4];   
	        double r23 = pose.R->data[5];   
	        double r31 = pose.R->data[6];   
	        double r32 = pose.R->data[7];   
	        double r33 = pose.R->data[8];  

	  //  double * quater = Rotation_Quaternion (r11,r12,r13,r21,r22,r23,r31,r32,r33);
            pose_msg.pose.orientation.w = time_c.toSec();
            pose_msg.pose.orientation.x = atan2(r32,r33) * 180 / M_PI;
            pose_msg.pose.orientation.y = atan2(-r31, sqrt(r32*r32+r33*r33)) * 180 / M_PI;
            pose_msg.pose.orientation.z = atan2(r21,r11) * 180 / M_PI;
	    pose_pub.publish(pose_msg);


            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        }
        apriltag_detections_destroy(detections);
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	double ttrack = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1).count();
	std::cout << "Tracking time cost is " << ttrack << "ms \n";
       imshow("Tag Detections", frame);
     //  imshow("Tag Detections undistort", un_gray);
        if (waitKey(30) >= 0)
            break;
       ros::spinOnce();  
       loop_rate.sleep();//与ros::Rate 
    }

    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tf);
    }


    getopt_destroy(getopt);
    cap.release();
    return 0;
}
