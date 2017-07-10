#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
public:
  // state vector
  Eigen::VectorXd x_ = Eigen::VectorXd(4);
  // state covariance matrix
  Eigen::MatrixXd P_ = Eigen::MatrixXd(4, 4);

  KalmanFilter() {}

  void Init(const Eigen::VectorXd& x0);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param dt Time between k and k+1 in s
   */
  void Predict(float dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param y The measurement error at k+1
   * @param H The measurement matrix
   * @param R The measurement covariance matrix
   */
  void Update(const Eigen::VectorXd &y, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

private:  
  // state transition matrix
  Eigen::MatrixXd F_ = Eigen::MatrixXd(4, 4);
  // process covariance matrix
  // Eigen::MatrixXd Q_ = Eigen::MatrixXd(4, 4);
};

#endif /* KALMAN_FILTER_H_ */
