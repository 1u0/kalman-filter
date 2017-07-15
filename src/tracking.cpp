#include <iostream>
#include <cmath>

#include "tracking.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

static const double PI = 3.14159265;

LidarModel::LidarModel() {
  R_ << 0.0225, 0,
        0, 0.0225;

  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;
}

void LidarModel::Init(KalmanFilter& kf, const VectorXd& z) const {
  float px = z[0];
  float py = z[1];
  VectorXd x0 = VectorXd(4);
  x0 << px, py, 0, 0;
  kf.Init(x0);
}

void LidarModel::Update(KalmanFilter& kf, const VectorXd& z) const {
  // State update using standard Kalman Filter equations.
  kf.Update(z - H_ * kf.x_, H_, R_);
}


RadarModel::RadarModel() {
  R_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
}

void RadarModel::Init(KalmanFilter& kf, const VectorXd& z) const {
  float ro = z[0];
  float theta = z[1];
  float ro_dot = z[2];
  float cos_theta = cos(theta), sin_theta = sin(theta);
  VectorXd x0 = VectorXd(4);
  x0 << ro * cos_theta, ro * sin_theta, 0, 0;//ro_dot * cos_theta, ro_dot * sin_theta;
  kf.Init(x0);
}

void RadarModel::Update(KalmanFilter& kf, const VectorXd& z) const {
  // State update using Extended Kalman Filter equations.
  const auto x = kf.x_;
  float px = x(0), py = x(1), vx = x(2), vy = x(3);

  float ro2 = px*px + py*py;
  float ro = sqrt(ro2);
  float ro3 = ro2 * ro;
  float theta = atan2(py, px);
  float ro_dot = ro > 0.0001 ? (px*vx + py*vy)/ro : 0.0;

  VectorXd z0 = VectorXd(3);
  z0 << ro, theta, ro_dot;

  // Jakobian measurement matrix
  MatrixXd Hj(3, 4);
  if (ro > 0.0001) {
    float diff = vx*py - vy*px;
    Hj << px/ro, py/ro, 0, 0,
          -py/ro2, px/ro2, 0, 0,
          diff*py/ro3, -diff*px/ro3, px/ro, py/ro;

  }
  VectorXd y = z - z0;
  if (y(1) > PI) {
    y[1] -= 2*PI;
  } else if (y(1) < -PI) {
    y[1] += 2*PI;
  }
  kf.Update(y, Hj, R_);
}

// Process a single measurement
void Tracking::Process(const MeasurementPackage& measurement) {
  if (0 == (sensors_ & measurement.sensor_type_)) {
    return;
  }

  // Compute the time elapsed between the current and previous measurements, expressed in seconds
  float dt = (measurement.timestamp_ - previous_timestamp_) / 1000000.0;
  cout << "dt = " << dt << endl;
  previous_timestamp_ = measurement.timestamp_;
  if (!is_initialized_ || dt > 1e+7) {
    cout << "Kalman Filter Initialization " << endl;
    switch (measurement.sensor_type_) {
      case MeasurementPackage::LASER:
        lidar_.Init(kf_, measurement.raw_measurements_);
        break;
      case MeasurementPackage::RADAR:
        radar_.Init(kf_, measurement.raw_measurements_);
        break;
    }

    is_initialized_ = true;
    return;
  }

  kf_.Predict(dt);

  // Measurement update
  switch (measurement.sensor_type_) {
    case MeasurementPackage::LASER:
      lidar_.Update(kf_, measurement.raw_measurements_);
      break;
    case MeasurementPackage::RADAR:
      radar_.Update(kf_, measurement.raw_measurements_);
      break;  
  }

  // cout << "x_ = " << kf_.x_ << endl;
  // cout << "P_ = " << kf_.P_ << endl;
}

VectorXd Tracking::GetEstimate() const {
  return kf_.x_;
}
