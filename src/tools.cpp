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
  double dist = sqrt(pow(px,2) + pow(py,2));
  double dist_2 = dist*dist;
  double dist_3 = dist_2*dist;
  double cross_mult = vx*py - vy*px;
  double px_dist = px/dist;
  double py_dist = py/dist;
  float tol = 0.0001;
  if (dist > tol){
    Hj(0,0) = px_dist;
    Hj(0,1) = py_dist;
    Hj(1,0) = -py/dist_2;
    Hj(1,1) = px/dist_2;
    Hj(2,0) = py*cross_mult/dist_3;
    Hj(2,1) = -px*cross_mult/dist_3;
    Hj(2,2) = px_dist;
    Hj(2,3) = py_dist;
  }
  return Hj;
}
