//This is a constant velocity extend kalman filter implementation

#include "filter.h"




//Definition

void cv_filter::init(Eigen::Vector3d& kmeasure){



    X.block<3,1>(0,0) = kmeasure;
    X.block<3,1>(3,0) = Eigen::MatrixXd::Zero(3,1);   
    X_pre = X; 

    F << 1 , 0 , 0 , dt ,  0 ,  0,
         0 , 1 , 0 , 0  , dt ,  0,
         0 , 0 , 1 , 0  ,  0 , dt,
         0 , 0 , 0 , 1  ,  0 ,  0,
         0 , 0 , 0 , 0  ,  1 ,  0,
         0 , 0 , 0 , 0  ,  0 ,  1; // F constant velocity 

    P << 0.01, 0   , 0   , 0   ,  0   ,  0,
         0   , 0.01, 0   , 0   ,  0   ,  0,
         0   , 0   , 0.01, 0   ,  0   ,  0,
         0   , 0   , 0   , 0.01,  0   ,  0,
         0   , 0   , 0   , 0   ,  0.01,  0,
         0   , 0   , 0   , 0   ,  0   ,  0.01;

    P_pre = P;

    Q << 0.01, 0   , 0   , 0   ,  0   ,  0  ,
         0   , 0.01, 0   , 0   ,  0   ,  0  ,
         0   , 0   , 0.01, 0   ,  0   ,  0  ,
         0   , 0   , 0   , 0.01,  0   ,  0  ,
         0   , 0   , 0   , 0   ,  0.01,  0  ,
         0   , 0   , 0   , 0   ,  0   ,  0.01;

    H << 1 , 0 , 0 , 0 , 0 ,  0,
         0 , 1 , 0 , 0 , 0 ,  0,
         0 , 0 , 1 , 0 , 0 ,  0,
         0 , 0 , 0 , 1 , 0 ,  0,
         0 , 0 , 0 , 0 , 1 ,  0,
         0 , 0 , 0 , 0 , 0 ,  1;

    K = Eigen::MatrixXd::Zero(N,M);

    
    




    R << 0.1 ,  0  , 0 ,  0  ,  0 ,  0 ,
          0  , 0.1 , 0 ,  0  ,  0 ,  0 ,
          0  ,  0  ,0.1,  0  ,  0 ,  0 ,
          0  ,  0  , 0 ,  0.05,  0 ,  0 ,
          0  ,  0  , 0 ,  0  , 0.05,  0 ,
          0  ,  0  , 0 ,  0  ,  0 , 0.05 ;

    //std::cout << X << std::endl<< F << std::endl<< P << std::endl<< Q << std::endl<< H << std::endl<< R << std::endl;
    
}


void cv_filter::predict(){

     X_pre = F * X ; // onestep predict state
     P_pre = F * P * F.transpose() + Q; //covariance predict

     //Record value
     X = X_pre;
     P = P_pre;
     pub_flag = 1;

  //   std::cout << X  << std::endl << F  << std::endl << P  << std::endl;
    
}

void cv_filter::update(Eigen::Matrix<double,cv_filter::M,1>& observe){
     Z =  observe;
     K =  P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
     X =  X_pre + K * ( Z - H * X_pre);
     P = (Eigen::MatrixXd::Identity(N,N) - K * H) * P_pre; 


     

}

 void cv_filter::receivedata(const geometry_msgs::PoseStamped::ConstPtr& msg){
      Eigen::Matrix<double,cv_filter::M,1> measurement;
      // There need to be compensate with attitude of UAV , if have yaw angle it will cause more error;
      measurement(0) = msg->pose.position.x;
      measurement(1) = msg->pose.position.y;
      measurement(2) = msg->pose.position.z; //compensate some offset
      Eigen::Vector3d p_measure = measurement.block<3,1>(0,0);
      


    // std::cout << measurement << std::endl; detect loss
      if(F_count > 10)
          loss = true;

      if (measurement.norm() == 0){ //detect lost
           F_count ++;
           return;    
      }

     //detect 
      else if (this->ini == 0){
           init(p_measure);
           this->ini = 1;
           pub_flag = 1;
           
      }
      else if (this->ini ==1 && p_measure.norm() != 0){
           position_data.push_back(std::make_pair(msg->header.stamp.toSec(),p_measure));

           if (sizeof(position_data) > 10){
                Eigen::Vector3d Velocity;
                double dt = position_data.back().first - position_data[position_data.size()-2].first;
                Velocity = (position_data.back().second - position_data[position_data.size()-2].second) / dt;
              //std::cout << "time is " << dt << std::endl<<"v====isss   " << Velocity << std::endl << "posi is  " << position_data.back().second 
              //   << "posi old is  " << position_data[position_data.size()-2].second << std::endl;
                measurement.block<3,1>(3,0) = Velocity;
           }


           loss = false;
           this->update(measurement);
           pub_flag = 1;
           F_count = 0;
      }

      
      
          

 }


void cv_filter::publishpose(const ros::Publisher& pub){
     if(pub_flag == 1){
          nav_msgs::Odometry pose_msg;
          pose_msg.header.stamp = ros::Time::now();
          pose_msg.header.frame_id = "filter_apriltag_tag";

          pose_msg.pose.pose.position.x = X(1);
          pose_msg.pose.pose.position.y = X(0) + 0.05;
          pose_msg.pose.pose.position.z = -X(2)- 0.02;
          pose_msg.pose.covariance[0] = P(1,1);
          pose_msg.pose.covariance[7] = P(0,0);
          pose_msg.pose.covariance[15] = P(2,2);
          pose_msg.pose.covariance[1] = 0;
          //rotation is comment not used in single drone
          /*Eigen::Matrix3d relative_rotation = relative_T.block<3,3>(0,0);
          Eigen::Quaternion<double> q_relative(relative_rotation);
          pose_msg.pose.pose.orientation.x = q_relative.x();
          pose_msg.pose.pose.orientation.y = q_relative.y();
          pose_msg.pose.pose.orientation.z = q_relative.z();
          pose_msg.pose.pose.orientation.w = q_relative.w();*/
          pose_msg.twist.twist.linear.x =  X(4);
          pose_msg.twist.twist.linear.y =  X(3);
          pose_msg.twist.twist.linear.z =  -X(5);

          if (loss == 1)
               pose_msg.pose.covariance[1] = 1;
          pub.publish(pose_msg);
          pub_flag = 0;
     }

      }
