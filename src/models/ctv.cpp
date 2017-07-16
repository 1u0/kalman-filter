#include "ctv.h"

#include <cmath>
#include "commons.h"

namespace CTV {

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace KF {

void Init(KalmanFilter& kf, const VectorXd& x) {
  kf.x_ = x;
  kf.P_ <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
}

void Update(KalmanFilter& kf, const VectorXd& y, const MatrixXd& H, const MatrixXd& R) {
  const MatrixXd PHt = kf.P_ * H.transpose();
  const MatrixXd S = H * PHt + R;
  const MatrixXd K = PHt * S.inverse();

  kf.x_ += K * y;
  kf.P_ -= K * H * kf.P_;
}

// State update using standard Kalman Filter equations.
void LinearUpdate(KalmanFilter& kf, const VectorXd& z, const MatrixXd& H, const MatrixXd& R) {
  Update(kf, z - H * kf.x_, H, R);
}

// State update using Extended Kalman Filter equations.
void ExtendedUpdate(KalmanFilter& kf, const VectorXd& z, const MatrixXd& R) {
  const auto x = kf.x_;
  float px = x(0), py = x(1), vx = x(2), vy = x(3);

  float ro2 = px*px + py*py;
  float ro = sqrt(ro2);
  float ro3 = ro2 * ro;
  float phi = atan2(py, px);
  float ro_dot = ro > 0.0001 ? (px*vx + py*vy)/ro : 0.0;

  // Predicted measurement
  VectorXd z0 = VectorXd(3);
  z0 << ro, phi, ro_dot;

  // Jakobian measurement matrix
  MatrixXd Hj(3, 4);
  if (ro > 0.0001) {
    float diff = vx*py - vy*px;
    Hj << px/ro, py/ro, 0, 0,
          -py/ro2, px/ro2, 0, 0,
          diff*py/ro3, -diff*px/ro3, px/ro, py/ro;

  }
  VectorXd y = z - z0;
  y(1) = NormalizeAngle(y(1));
  Update(kf, y, Hj, R);
}

} // namespace KF


Model::Model() {
  F_ <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;  

  lidar_R_ <<
    0.0225, 0, // std_px = 0.15 m
    0, 0.0225; // std_py = 0.15 m
  lidar_H_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;

  radar_R_ <<
    0.09, 0, 0,   // std_ro = 0.3 m
    0, 0.0009, 0, // std_phi = 0.03 rad
    0, 0, 0.09;   // std_ro_dot = 0.3 m/s
}

void Model::Init(MeasurementPackage::SensorType type, const VectorXd& z) {
  switch (type) {
    case MeasurementPackage::LASER: {
      double px = z(0);
      double py = z(1);
      VectorXd x0(4);
      x0 << px, py, 0, 0;
      KF::Init(kf_, x0);
      break;
    }
    case MeasurementPackage::RADAR: {
      double ro = z(0);
      double phi = z(1);
      double ro_dot = z(2);
      double cos_phi = cos(phi), sin_phi = sin(phi);
      VectorXd x0 = VectorXd(4);
      x0 << ro * cos_phi, ro * sin_phi, 0, 0;//ro_dot * cos_phi, ro_dot * sin_phi;
      KF::Init(kf_, x0);
      break;
    }
  }
}

void Model::Update(MeasurementPackage::SensorType type, const VectorXd& z) {
  switch (type) {
    case MeasurementPackage::LASER:
      KF::LinearUpdate(kf_, z, lidar_H_, lidar_R_);
      break;
    case MeasurementPackage::RADAR:
      KF::ExtendedUpdate(kf_, z, radar_R_);
      break;
  }
}

void Model::Predict(double dt) {
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

  kf_.x_ = F_ * kf_.x_; // + u;
  kf_.P_ = F_ * kf_.P_ * F_.transpose() + Q;
}

VectorXd Model::GetEstimate() const {
  return kf_.x_;
}

} // namespace CTV
