//This is a constant velocity extend kalman filter implementation

#include "filter.h"




//Definition

void cv_filter::init(){
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

    Q << 0.01, 0   , 0   , 0   ,  0   ,  0  ,
         0   , 0.01, 0   , 0   ,  0   ,  0  ,
         0   , 0   , 0.01, 0   ,  0   ,  0  ,
         0   , 0   , 0   , 0.01,  0   ,  0  ,
         0   , 0   , 0   , 0   ,  0.01,  0  ,
         0   , 0   , 0   , 0   ,  0   ,  0.01;

    H << 1 , 0 , 0 , dt ,  0 ,  0,
         0 , 1 , 0 , 0  , dt ,  0,
         0 , 0 , 1 , 0  ,  0 , dt;

    
    




    R << 0.1 ,  0 ,
          0  , 0.1;

    
    
}


void cv_filter::predict(){

     X_pre = F * X ; // onestep predict state

     P_pre = F * P * P + Q; //covariance predict
     
}

void cv_filter::update(Eigen::Vector3d& observe){

     K =  P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
     X =  X_pre + K * ( Z - H * X_pre);
     P = (Eigen::MatrixXd::Identity(N,N) - K * H) * P_pre; 

}

