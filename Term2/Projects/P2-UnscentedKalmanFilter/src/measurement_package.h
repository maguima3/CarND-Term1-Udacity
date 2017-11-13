#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
  long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  //cointains measurement
  //for lidar: px, py
  //for radar: ro,theta, ro_dot
  Eigen::VectorXd raw_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
