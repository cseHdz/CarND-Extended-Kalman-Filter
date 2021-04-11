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
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
