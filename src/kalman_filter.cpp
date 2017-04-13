#include "kalman_filter.h"

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
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // The general Kalman filter update equations are computed below
  MatrixXd y = z - H_*x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd P_Ht = P_*Ht;
  MatrixXd S = H_*P_Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_Ht*Si;
  x_ += K*y;
  P_ = (MatrixXd::Identity(4,4) - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Creating the non-linear H(x) function for the first step
  // of error calculation
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  VectorXd H_fun = VectorXd(3);
  double dist = sqrt(pow(px,2) + pow(py,2));
  H_fun<< dist,
          atan2(py,px),
          (px*vx + py*vy)/dist;
  //VectorXd H_fun = H_*x_;
  VectorXd y = z - H_fun;
  // Using atan2 to get the correct difference in angles
  double dphi = atan2(sin(z(1)),cos(z(1))) - atan2(sin(H_fun(1)),cos(H_fun(1)));
  y(1) = dphi;
  // rest of the prediction steps
  MatrixXd Ht = H_.transpose();
  MatrixXd P_Ht = P_*Ht;
  MatrixXd S = H_*P_Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_Ht*Si;
  x_ += K*y;
  P_ = (MatrixXd::Identity(4,4) - K*H_)*P_;
}
