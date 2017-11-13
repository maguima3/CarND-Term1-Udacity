#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

static const string out_file_name_ = "../results/output.txt";

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * Writes result in output file
  */
  void WriteResult(UKF::UKF ukf, MeasurementPackage::MeasurementPackage meas_package, VectorXd gt_values);

};

#endif /* TOOLS_H_ */