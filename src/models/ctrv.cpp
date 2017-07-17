#include "ctrv.h"

#include <iostream>


#include <cmath>
#include "commons.h"

namespace CTRV {

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace KF {

void Init(KalmanFilter& kf, const VectorXd& x) {
  kf.x_ = x;
  kf.P_ <<
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 10, 0, 0,
    0, 0, 0, 10, 0,
    0, 0, 0, 0, 10;
}

static
MatrixXd GenerateSigmaPoints(const VectorXd& x, const MatrixXd& P) {
  const int n_x = x.rows();
  const double lambda = 3 - n_x;
  assert(n_x == P.rows() && n_x == P.cols());

  // Sigma points matrix: columns are points.
  // X_k|k = [x_k|k, x_k|k + sqrt((lambda + n_x)*P_k|k), x_k|k - sqrt((lambda + n_x)*P_k|k)]
  MatrixXd Xsig = MatrixXd(n_x, 1 + 2*n_x);

  // Square root of matrix P.
  MatrixXd A = P.llt().matrixL();
  A *= sqrt(lambda + n_x);

  Xsig.col(0) = x;
  for (int i = 0; i < n_x; ++i) {
      Xsig.col(i+1) = x + A.col(i);
      Xsig.col(i+1+n_x) = x - A.col(i);
  }
  return Xsig;
}

// Calculate weighted mean and weighted covariance matrix
template<typename Normalize>
void Eval(const VectorXd& weights, const MatrixXd& Xsig, VectorXd& x, MatrixXd& K,
    const Normalize& normalize)
{
  const int n_x = Xsig.rows();
  const int count = weights.rows();
  assert(count == Xsig.cols());

  const VectorXd mean = Xsig * weights;
  MatrixXd covariance(n_x, n_x);
  covariance.fill(0.0);
  for (int i = 0; i < count; ++i) {
    const VectorXd xd = normalize(Xsig.col(i) - mean);
    covariance += weights(i) * xd * xd.transpose();
  }
  x = mean;
  K = covariance;
}

// Update using unscented Kalman Filter equations
template<typename Predict, typename NormalizeState, typename NormalizeMeasurement>
void Update(KalmanFilter& kf, const VectorXd& weights, const MatrixXd& Xsig, const VectorXd& z, const MatrixXd& R,
    const Predict& predict, const NormalizeState& normalizeState, const NormalizeMeasurement& normalizeMeasurement)
{
  const int n_x = Xsig.rows();
  const int n_z = z.rows();
  const int count = weights.rows();
  assert(n_z == R.rows() && n_z == R.cols());
  assert(count == Xsig.cols());

  // Transform sigma points into measurement space
  MatrixXd Zsig(n_z, count);
  for (int i = 0; i < count; ++i) {
    Zsig.col(i) = predict(Xsig.col(i));
  }

  VectorXd z_pred(n_z);
  MatrixXd S(n_z, n_z);
  Eval(weights, Zsig, z_pred, S, normalizeMeasurement);
  S += R;

  // Cross corelation matrix.
  MatrixXd Tc(n_x, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < count; ++i) {
    VectorXd z_diff = normalizeMeasurement(Zsig.col(i) - z_pred);
    VectorXd x_diff = normalizeState(Xsig.col(i) - kf.x_);

    Tc += weights(i) * x_diff * z_diff.transpose();
  }
  // Kalman gain K.
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = normalizeMeasurement(z - z_pred);
  kf.x_ += K * z_diff;
  kf.P_ -= K * S * K.transpose();
}

} // namespace KF

// TODO: make method, some fields are model specific
static
void AugmentModel(const KalmanFilter& kf, VectorXd& x_aug, MatrixXd& P_aug) {
  const VectorXd& x = kf.x_;
  const MatrixXd& P = kf.P_;
  const int n_x = x.rows();
  // const int n_aug = n_x + 2;
  //Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;
  //Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  // VectorXd x(n_aug);
  // MatrixXd P_aug(n_aug, n_aug);
  x_aug.head(n_x) = x;
  x_aug(n_x+0) = 0.0;
  x_aug(n_x+1) = 0.0;
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x, n_x) = P;
  P_aug(n_x+0, n_x+0) = std_a * std_a;
  P_aug(n_x+1, n_x+1) = std_yawdd * std_yawdd;
}

static
MatrixXd GenerateSigmaPoints(const KalmanFilter& kf) {
  VectorXd x_aug(7);
  MatrixXd P_aug(7, 7);
  AugmentModel(kf, x_aug, P_aug);
  return KF::GenerateSigmaPoints(x_aug, P_aug);
}

static
VectorXd PredictModel(const VectorXd& x_aug, double dt) {
  const int n_x = 5;
  VectorXd x(n_x);
  x = x_aug.head(n_x);
  double v = x_aug(2), yaw = x_aug(3), yawd = x_aug(4), nu_a = x_aug(5), nu_yawdd = x_aug(6);
  if (fabs(yawd) < 0.0001) {
      x(0) += v*dt*cos(yaw);
      x(1) += v*dt*sin(yaw);
      x(3) += yawd*dt;
  } else {
      x(0) += v * (sin(yaw + yawd*dt) - sin(yaw)) / yawd;
      x(1) += v * (-cos(yaw + yawd*dt) + cos(yaw)) / yawd;
      x(3) += yawd*dt;
  }
  double dt2 = dt*dt/2;
  x(0) += nu_a*cos(yaw) * dt2;
  x(1) += nu_a*sin(yaw) * dt2;
  x(2) += nu_a * dt;
  x(3) += nu_yawdd * dt2;
  x(4) += nu_yawdd * dt;
  
  return x;
}

static
VectorXd NormalizeState(VectorXd x) {
  x(3) = NormalizeAngle(x(3));
  return x;
}

static
VectorXd GetWeights(int n_x, double lambda) {
  assert(n_x > 0);
  const int count = 1 + 2*n_x;
  VectorXd weights(count);
  weights(0) = lambda / (lambda + n_x);

  const double coef = 0.5 / (lambda + n_x);
  for (int i = 1; i < count; ++i) {
    weights(i) = coef;
  }
  return weights;
}


void LidarUpdate(KalmanFilter& kf, const VectorXd& z, const VectorXd& weights, const MatrixXd& Xsig_pred, const MatrixXd& R) {
  KF::Update(kf, weights, Xsig_pred, z, R,
    // Predict measurement from state vector.
    [](const VectorXd& x) {
      double px = x(0), py = x(1);

      VectorXd z(2);
      z << px, py;
      return z;
    },
    NormalizeState,
    [](const VectorXd& z) {
      return z;
    });
}

void RadarUpdate(KalmanFilter& kf, const VectorXd& z, const VectorXd& weights, const MatrixXd& Xsig_pred, const MatrixXd& R) {
  KF::Update(kf, weights, Xsig_pred, z, R,
    // Predict measurement from state vector.
    [](const VectorXd& x) {
      double px = x(0), py = x(1), v = x(2), yaw = x(3);
      double vx = v * cos(yaw);
      double vy = v * sin(yaw);

      double ro = sqrt(px*px + py*py);
      double phi = atan2(py, px);
      double ro_dot = ro > 0.0001 ? (px*vx + py*vy)/ro : 0.0;
      VectorXd z(3);
      z << ro, phi, ro_dot;
      return z;
    },
    NormalizeState,
    [](VectorXd z) {
      z(1) = NormalizeAngle(z(1));
      return z;
    });
}

Model::Model()
  : weights_(GetWeights(7, 3 - 7))
  , Xsig_pred_(5, 1+2*7)
{
  lidar_R_ <<
    0.0225, 0, // std_px = 0.15 m
    0, 0.0225; // std_py = 0.15 m

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
      VectorXd x0(5);
      x0 << px, py, 0, 0, 0;
      KF::Init(kf_, x0);
      break;
    }
    case MeasurementPackage::RADAR: {
      double ro = z(0);
      double phi = z(1);
      double ro_dot = z(2);
      double cos_phi = cos(phi), sin_phi = sin(phi);
      VectorXd x0 = VectorXd(5);
      x0 << ro * cos_phi, ro * sin_phi, 0, 0, 0;
      KF::Init(kf_, x0);
      break;
    }
  }
}

void Model::Update(MeasurementPackage::SensorType type, const VectorXd& z) {
  //cout << "Update" << endl;
  switch (type) {
    case MeasurementPackage::LASER:
      LidarUpdate(kf_, z, weights_, Xsig_pred_, lidar_R_);
      break;
    case MeasurementPackage::RADAR:
      RadarUpdate(kf_, z, weights_, Xsig_pred_, radar_R_);
      break;
  }
  // cout << "Update." << endl;
}

void Model::Predict(double dt) {
  // cout << "Predict" << endl;
  MatrixXd Xsig_aug = GenerateSigmaPoints(kf_);
  // cout << "GenerateSigmaPoints." << endl;
  // cout << "Xsig_aug " << Xsig_aug << endl;

  const int n_x = kf_.x_.rows();
  const int count = Xsig_aug.cols();

  // Transform augmented sigma points into target space according to model.
  MatrixXd Xsig_pred(n_x, count);
  for (int i = 0; i < count; ++i) {
    Xsig_pred.col(i) = PredictModel(Xsig_aug.col(i), dt);
  }
  Xsig_pred_ = Xsig_pred;

  // Update state
  KF::Eval(weights_, Xsig_pred_, kf_.x_, kf_.P_, NormalizeState);
  // cout << "Predict." << endl;
}

VectorXd Model::GetEstimate() const {
  // cout << "Estimate" << endl;
  const VectorXd& x = kf_.x_;
  double px = x(0), py = x(1), v = x(2), yaw = x(3);
  double vx = v * cos(yaw);
  double vy = v * sin(yaw);
  VectorXd r(4);
  r << px, py, vx, vy;
  // cout << "Estimate." << endl;
  return r;
}

} // namespace CTRV