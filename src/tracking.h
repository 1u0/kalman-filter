#ifndef TRACKING_H_
#define TRACKING_H_

#include <iostream>
#include "Eigen/Dense"

#include "measurement_package.h"

template<typename Model>
class Tracking {
public:
  Tracking() {}

  // Process a single measurement
  void Process(const MeasurementPackage& measurement) {
    if (0 == (sensors_ & measurement.sensor_type_)) {
      return;
    }

    // Compute the time elapsed between the current and previous measurements, expressed in seconds
    float dt = (measurement.timestamp_ - previous_timestamp_) / 1000000.0;
    std::cout << "dt = " << dt << ", " << measurement.timestamp_ << std::endl;
    previous_timestamp_ = measurement.timestamp_;
    if (!is_initialized_ || dt > 1e+5) {
      std::cout << "Initialization " << std::endl;
      model_.Init(measurement.sensor_type_, measurement.raw_measurements_);
      is_initialized_ = true;
      return;
    }
    model_.Predict(dt);
    model_.Update(measurement.sensor_type_, measurement.raw_measurements_);
  }

  // Returns 4D vector of (x, y, vx, vy).
  Eigen::VectorXd GetEstimate() const {
    return model_.GetEstimate();
  }

  // Bitmask of enabled sensors
  long sensors_ = MeasurementPackage::LASER | MeasurementPackage::RADAR;
private:
  bool is_initialized_ = false;
  long long previous_timestamp_ = 0;
  
  Model model_;
};

#endif//TRACKING_H_