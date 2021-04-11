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
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  // Common terms across Jacobian matrix
  float c1 = px*px+py*py; // (px^2 + py^2)
  float c2 = sqrt(c1); // (px^2 + py^2)^1/2

  float ro = c2;
  float phi = arctan2(py, px);
  float ro_dot = (px*vx + py*vy)/ro;
  
  VectorXd hx = VectorXd(3);
  
  hx << ro, phi, ro_dot;
  
  VectorXd z_pred = hx * x_;
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
  P_ = (I - K * Hj) * P_;
 
}
