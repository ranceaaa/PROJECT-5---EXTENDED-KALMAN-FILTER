#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1,0,0,0,
	      0,1,0,0;
  

  ekf_.F_ = MatrixXd(4, 4);
  
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<  1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 100;
  ekf_.Q_ = MatrixXd(4, 4);
  
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
  /**
   * Initialization
   */
  int type = measurement_pack.sensor_type_;
    if(type == 0)
    {
	cout<<"LASER";
    }
    if(type == 1)
    {
	cout<<"RADAR";
    }
    cout<<endl;
  if (!is_initialized_) 
  {
    cout<<"INITIALIZING"<<endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1,1,1,1;
    //cout<<ekf_.x_<<endl;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
	float rho = measurement_pack.raw_measurements_[0]; // range
	float phi = measurement_pack.raw_measurements_[1]; // bearing
	float rho_dot = measurement_pack.raw_measurements_[2]; // rate
        cout<<"phi : "<<phi<<endl;
        // phi normalization -2Pi + 2Pi
        while(phi > M_PI)
	{
	    phi -= 2.0*M_PI;
	}
	while(phi < -M_PI)
        {
	    phi += 2.0*M_PI;
	}
	if(phi > M_PI || phi < -M_PI)
	{
	    cout<<"PHI in afara "<<endl;
	}


	float x = rho * cos(phi);
	float y = rho * sin(phi);
	float vx = rho_dot * cos(phi);
	float vy = rho_dot * sin(phi);
	ekf_.x_ << x,y,vx,vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      // TODO: Initialize state.
	float x = measurement_pack.raw_measurements_(0);
	float y = measurement_pack.raw_measurements_(1);
        


        ekf_.x_ << x,y,0,0;
    }
    float val = 0.0001;
    if(fabs(ekf_.x_(0))< val and fabs(ekf_.x_(1)) < val)
    {
        ekf_.x_(0) = val;
        ekf_.x_(1) = val;
    }
    cout << "EKF: " << endl;
    cout<<ekf_.x_<<endl;
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
  
    return;
  }
  float dt = (measurement_pack.timestamp_ - previous_timestamp_);

  dt = dt/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  
  ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float dt_4_4 = dt_4 / 4;
  float dt_3_2 = dt_3 / 2;

  ekf_.Q_ << dt_4_4*noise_ax, 0,               dt_3_2*noise_ax, 0,
             0,               dt_4_4*noise_ay, 0,               dt_3_2*noise_ay,
             dt_3_2*noise_ax, 0,               dt_2*noise_ax,   0,
             0,               dt_3_2*noise_ay, 0,               dt_2*noise_ay;
  ekf_.Predict();
  
  /**
   * Update
   */
 
  

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  { Tools tools;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else 
  {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
