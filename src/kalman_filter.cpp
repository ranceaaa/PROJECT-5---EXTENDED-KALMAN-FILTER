#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}


void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  
}

void KalmanFilter::Predict() {
    x_ = F_ * x_ ;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   VectorXd y = z - H_ * x_;
   MatrixXd Ht = H_.transpose();
   MatrixXd S = H_ * P_ * Ht + R_;
   
   MatrixXd Si = S.inverse();
   MatrixXd k = P_ * Ht * Si;
   long x_size = x_.size();
   MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
   x_ = x_ + (k * y);
   P_ = (I_ - k * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    long x_size = x_.size();
    MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
    float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    float theta = atan2(x_(1),x_(0));
    float rho_dot;
    float val = 0.0001;
    if(fabs(theta)< val){
        rho_dot = 0; 
    }else{

    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
    }
    VectorXd h = VectorXd(3);
    h << rho, theta, rho_dot;
    VectorXd y = z - h;
    while(y(1) > M_PI)
	{
	    y(1) -= 2.0*M_PI;
	}
	while(y(1) < -M_PI)
        {
	    y(1) += 2.0*M_PI;
	}
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ *Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd k = P_ * Ht * Si;
    x_ = x_ + (k * y);
    P_ = (I_ - k * H_) * P_;
}
