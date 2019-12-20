#ifndef filter_H
#define filter_H
//This is a constant velocity extend kalman filter implementation
/**
  * Initializes an EKF structure.
  * @param ekf pointer to EKF structure to initialize
  * @param n number of state variables
  * @param m number of observables
  *
  * <tt>ekf</tt> should be a pointer to a structure defined as follows, where <tt>N</tt> and </tt>M</tt> are 
  * constants:
  * <pre>
        int n;           // number of state values 
        int m;           // number of observables 
        double x[N];     // state vector
        double P[N][N];  // prediction error covariance
        double Q[N][N];  // process noise covariance 
        double R[M][M];  // measurement error covariance
        double G[N][M];  // Kalman gain; a.k.a. K
        double F[N][N];  // Jacobian of process model
        double H[M][N];  // Jacobian of measurement model
        double Ht[N][M]; // transpose of measurement Jacobian
        double Ft[N][N]; // transpose of process Jacobian
        double Pp[N][N]; // P, post-prediction, pre-update
        double fx[N];   // output of user defined f() state-transition function
        double hx[M];   // output of user defined h() measurement function
      &nbsp; // temporary storage
        double tmp0[N][N];
        double tmp1[N][Msta];
        double tmp2[M][N];
        double tmp3[M][M];
        double tmp4[M][M];
        double tmp5[M]; 
    * </pre>
  */
#include <iostream>
#include <vector>


#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

//Declaration
class cv_filter{
  public:
    cv_filter(){};
    bool  ini  = false;
    bool  loss = false;
    bool  pub_flag  = false;
    const double dt = 1.0f/100;
    const static int N = 6;
    const static int M = 3;


    void init(Eigen::Vector3d& kmeasure);
    void predict();
    void update(Eigen::Vector3d& observe);
    void receivedata(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void publishpose(const ros::Publisher& pub);

    


    Eigen::Matrix<double,N,1> X,X_pre,X_temp; //state
    Eigen::Matrix<double,N,N> F,P,Q,P_pre,P_temp; //state transformtion matrix and covariance matrix and process noise
    Eigen::Matrix<double,M,N> H; //observation matrix
    Eigen::Matrix<double,N,M> K; //gain
    Eigen::Matrix<double,M,M> R; // covariance of observation
    Eigen::Matrix<double,M,1> Z; // observation
    


};




#endif