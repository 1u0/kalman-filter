#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

Tools::RMSE::RMSE() {
  square_error_ << 0, 0, 0, 0;
}

VectorXd Tools::RMSE::Update(const VectorXd& estimation, const VectorXd& ground_truth) {
  VectorXd diff = estimation - ground_truth;
  diff = diff.array() * diff.array();

  square_error_ += diff;
  ++count_;

  VectorXd mse = square_error_ / count_;
  return mse.array().sqrt();
}
