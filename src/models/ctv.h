#ifndef CTV_EKF_H_
#define CTV_EKF_H_

#include "../Eigen/Dense"

#include "../kalman_filter.h"
#include "../measurement_package.h"

// Constant Turn and Velocity model, using extended Kalman Filter (for radar).
// State vector:
//  [px, py, vx, vy]

namespace CTV {

class Model {
  // State transition matrix
  Eigen::MatrixXd F_ = Eigen::MatrixXd(4, 4);

  KalmanFilter kf_ = KalmanFilter(4);

  // Lidar measurements
  Eigen::MatrixXd lidar_R_ = Eigen::MatrixXd(2, 2);
  Eigen::MatrixXd lidar_H_ = Eigen::MatrixXd(2, 4);
  // Radar measurements
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

} // namespace CTV

#endif//CTV_EKF_H_