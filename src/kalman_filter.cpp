#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void KalmanFilter::Init(const VectorXd& x0) {
  x_ = x0;
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
}

void KalmanFilter::Predict(float dt) {
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Update the state transition matrix F
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // Update the process covariance matrix Q
  //  acceleration noise components
  float noise_ax = 9.0, noise_ay = 9.0;
  MatrixXd Q(4, 4);
  Q << noise_ax * dt_4/4, 0, noise_ax * dt_3/2, 0,
       0, noise_ay * dt_4/4, 0, noise_ay * dt_3/2,
       noise_ax * dt_3/2, 0, noise_ax * dt_2, 0,
       0, noise_ay * dt_3/2, 0, noise_ay * dt_2;

  x_ = F_ * x_; // + u;
  P_ = F_ * P_ * F_.transpose() + Q;
}

void KalmanFilter::Update(const VectorXd& y, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
  const MatrixXd PHt = P_ * H.transpose();
  const MatrixXd S = H * PHt + R;
  const MatrixXd K = PHt * S.inverse();

  x_ += K * y;
  P_ -= K * H * P_;
}
