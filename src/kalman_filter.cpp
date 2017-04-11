#include "kalman_filter.h"
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd y = z - H_*x_;
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();
  x_ += K*y;
  P_ = (MatrixXd::Identity(4,4) - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  VectorXd H_fun = VectorXd(3);
  H_fun << sqrt(pow(px,2) + pow(py,2)),
            atan2(sin(py/px),cos(py/px)),
            (px*vx + py*vy)/sqrt(pow(px,2) + pow(py,2));
  VectorXd y = VectorXd::Zero(z.size());
  y(0) = z(0) - H_fun(0);
  y(2) = z(2) - H_fun(2);
  double dphi = atan2(sin(z(1)),cos(z(1))) - atan2(sin(H_fun(1)),cos(H_fun(1)));
  while (dphi < 0){
    dphi += 2*M_PI;
  }
  y(1) = dphi;
  //VectorXd y = z - H_*x_;
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_*H_.transpose()*S.inverse();
  x_ += K*y;
  P_ = (MatrixXd::Identity(4,4) - K*H_)*P_;
}
