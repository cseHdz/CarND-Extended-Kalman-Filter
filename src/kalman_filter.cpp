#include "kalman_filter.h"
#include "tools.h"

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
  
  x_ = x_in; // Object State
  P_ = P_in; // Object Covariance Matrix
  F_ = F_in; // State Transition Matrix
  H_ = H_in; // Measurement Matrix
  R_ = R_in; // Measurement Covariance Matrix 
  Q_ = Q_in; // Process Covariance Matrix
}

void KalmanFilter::Predict() {
  
  // Predict Equations LIDAR
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  // Linear Motion - Equations for LIDAR Measurement
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred; // y = z - Hx'
  MatrixXd Ht = H_.transpose(); 
  MatrixXd S = H_ * P_ * Ht + R_; // S = HP'H^T + R
  MatrixXd Si = S.inverse(); 
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si; // K = P'H^TS^-1

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  // Non-linear - Equations for RADAR Measurement
  // Assuming vector z has already been converted from polar coordinates
  
  // Get the Jacobian Matrix 

  
  VectorXd z_pred = Hj * x_;
  VectorXd y = z - z_pred; // y = z - Hx'
  MatrixXd Ht = Hj.transpose(); 
  MatrixXd S = Hj * P_ * Ht + R_; // S = HP'H^T + R
  MatrixXd Si = S.inverse(); 
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si; // K = P'H^TS^-1

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
 
}
