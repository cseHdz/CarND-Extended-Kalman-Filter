#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  // Implementation of function as per CarND Lectures
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // Sum of squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    // Error: difference between predicted minus actual for each obsevation
    VectorXd residual = estimations[i] - ground_truth[i];

    // Squared error
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // Mean Squared Error
  rmse = rmse/estimations.size();

  // Root Mean Squared Error
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  // Implementation of function as per CarND Lectures
  
  MatrixXd Hj(3,4);
  
  // Poisition and Velocity State
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Common terms across Jacobian matrix
  float c1 = px*px+py*py; // (px^2 + py^2)
  float c2 = sqrt(c1); // (px^2 + py^2)^1/2
  float c3 = (c1*c2); // (px^2 + py^2)^3/2

  // Ensure common term in denominator (px^2 + py^2) is not zero
  // Otherwise - divison by zero error
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // Compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
