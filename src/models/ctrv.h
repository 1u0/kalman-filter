#ifndef CTRV_UKF_H_
#define CTRV_UKF_H_

#include "../Eigen/Dense"

#include "../kalman_filter.h"
#include "../measurement_package.h"

// Constant Turn Rate and Velocity model, using unscented Kalman Filter.
// State vector:
//  [px, py, v, yaw, yawd]

namespace CTRV {

class Model {
  Eigen::VectorXd weights_;
  Eigen::MatrixXd Xsig_pred_;

  KalmanFilter kf_ = KalmanFilter(5);

  // Lidar measurements
  Eigen::MatrixXd lidar_R_ = Eigen::MatrixXd(2, 2);
  // Radar measurement
  Eigen::MatrixXd radar_R_ = Eigen::MatrixXd(3, 3);

public:
  Model();

  void Init(MeasurementPackage::SensorType type, const Eigen::VectorXd& m);
  void Update(MeasurementPackage::SensorType type, const Eigen::VectorXd& m);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param dt Time between k and k+1 in s
   */
  void Predict(double dt);

  Eigen::VectorXd GetEstimate() const;
};

} // namespace CTRV

#endif//CTRV_UKF_H_