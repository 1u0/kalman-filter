#ifndef TRACKING_H_
#define TRACKING_H_

#include "Eigen/Dense"

#include "measurement_package.h"
#include "kalman_filter.h"

class LidarModel {
private:
  // Measurement covariance matrix
  Eigen::MatrixXd R_ = Eigen::MatrixXd(2, 2);
  // Measurement matrix
  Eigen::MatrixXd H_ = Eigen::MatrixXd(2, 4);

public:
  LidarModel();

  void Init(KalmanFilter& kf, const Eigen::VectorXd& z) const;
  void Update(KalmanFilter& kf, const Eigen::VectorXd& z) const;
};

class RadarModel {
private:
  // Measurement covariance matrix
  Eigen::MatrixXd R_ = Eigen::MatrixXd(3, 3);

public:
  RadarModel();

  void Init(KalmanFilter& kf, const Eigen::VectorXd& z) const;
  void Update(KalmanFilter& kf, const Eigen::VectorXd& z) const;
};


class Tracking {
public:
  Tracking() {}

  void Process(const MeasurementPackage& measurement);
  KalmanFilter kf_;

private:
  bool is_initialized_ = !false;
  long previous_timestamp_ = 0;

  LidarModel lidar_;
  RadarModel radar_;
};

#endif /* TRACKING_H_ */