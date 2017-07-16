#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

struct KalmanFilter {
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  KalmanFilter(int n_x) : x_(n_x), P_(n_x, n_x) {}
};

#endif//KALMAN_FILTER_H_
