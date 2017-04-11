#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if ((estimations.size() < 0) &&(estimations.size() != ground_truth.size())){
    std::cout<<"RMSE cannot be calculated, please check vector sizes\n";
    return rmse;
  }

  for (size_t i = 0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Average rmse
  rmse = rmse/estimations.size();

  // Taking square root of individual elements
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);
  MatrixXd Hj = MatrixXd::Zero(3,4);
  double dist = pow(pow(px,2) + pow(py,2),0.5);
  float tol = 0.0001;
  if (dist > tol){
    Hj(0,0) = px/dist;
    Hj(0,1) = py/dist;
    Hj(1,0) = -py/pow(dist,2);
    Hj(1,1) = px/pow(dist,2);
    Hj(2,0) = py*(vx*py - vy*px)/pow(dist,3);
    Hj(2,1) = px*(vy*px - vx*py)/pow(dist,3);
    Hj(2,2) = px/dist;
    Hj(2,3) = py/dist;
  }
  return Hj;
}
