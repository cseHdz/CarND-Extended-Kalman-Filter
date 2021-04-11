#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;


  // State Covariance Matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  
  // State Transition Matrix (F_)
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
  		     0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  
  // Measurement Matrix - Lidar
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    cout << "EKF: " << endl;
    
    // Measurement Vector
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      // Converting polar coordinates to cartesian coordinates.
      
      float ro = measurement_pack.raw_measurements_[0];// Range (Distance from vehicle to pedestrian;
      float phi = measurement_pack.raw_measurements_[1]; //Bearing (angle between ro and x;
      float ro_dot = measurement_pack.raw_measurements_[2]; //Radial Velocity (ro rate of change)

      float px = ro * cos(phi);
      float py = ro * sin(phi);
      
      float vx = ro_dot * cos(phi);
      float vy = ro_dot * sin(phi);
      
      // Ensure the Jacobian can be computed - Avoid division by zero
      if (fabs(px) < 0.0001) {
        px = 0.0001;
      }
      
      if (fabs(py) < 0.0001) {
        py = 0.0001;
      }
      
      ekf_.x_ << px, py, vx, vy;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      
      // Only initial State is known - set velocity to zero
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

    }
    
    // No previous timestamp - Done Initializing
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  
  // Time elapsed between measurements -> Converted to Seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Update the State Transition Matrix with elapsed time
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
    
  // Acceleration noise
  float noise_ax = 9;
  float noise_ay = 9;
  
  // Update the Process Noise Covariance Matrix
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    
    // Calculate the Kalman Filter using The Jacobian using the previous state
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
