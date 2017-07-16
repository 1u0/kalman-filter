#ifndef MODELS_COMMONS_H_
#define MODELS_COMMONS_H_

inline
double NormalizeAngle(double angle) {
  const double PI = 3.14159265;
  if (angle > PI) {
    return angle - 2*PI;
  }
  if (angle < -PI) {
    return angle + 2*PI;
  }
  return angle;	
}

#endif//MODELS_COMMONS_H_