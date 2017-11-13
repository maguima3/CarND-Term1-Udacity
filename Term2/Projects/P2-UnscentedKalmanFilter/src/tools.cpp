#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse = VectorXd(4);
  rmse.fill(0.0);
  
  // check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size() || estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i=0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

void Tools::WriteResult(UKF ukf, MeasurementPackage meas_package, VectorXd gt_values) {
	//Defines output file, where all results will be saved
  std::ofstream out_file_(out_file_name_.c_str(), std::ofstream::out | ios::app);
  if (!out_file_.is_open()) {
    cerr << "Cannot open output file: " << out_file_name_ << endl;
    exit(EXIT_FAILURE);
  }

  //reads first line
  std::ifstream file(out_file_name_);
  std::string line;
  std::getline(file, line);
  //if does not contain the header, write it
  if (line.find("p1est\t") == std::string::npos) {

  	// column names for output file
	  out_file_ << "p1est" << "\t"; //px estimated
	  out_file_ << "p2est" << "\t"; //py est
	  out_file_ << "vest" << "\t"; //v est
	  out_file_ << "yawest" << "\t"; //yaw est
	  out_file_ << "yawrateest" << "\t"; //yaw_rate est
	  out_file_ << "p1meas" << "\t"; //px meas
	  out_file_ << "p2meas" << "\t"; //py meas
	  out_file_ << "p1" << "\t"; //x true
	  out_file_ << "p2" << "\t"; //y true
	  out_file_ << "v1_gt" << "\t"; //vx true
	  out_file_ << "v2_gt" << "\t"; //vy true
	  out_file_ << "NIS" << "\t";
	  out_file_ << "sensor_type" << "\n"; //sensor type
  

  } else { // already contains the header
	 	
	  //writes at the enf of the file
	  out_file_.seekp(0, ios::end);


	  //Save estimated results in output file
	  out_file_ << ukf.x_(0) << "\t"; // px - estimated
	  out_file_ << ukf.x_(1) << "\t"; // py - estimated
	  out_file_ << ukf.x_(2) << "\t"; // v - estimated
	  out_file_ << ukf.x_(3) << "\t"; // yaw_angle - estimated
	  out_file_ << ukf.x_(4) << "\t"; // yaw_rate - estimated

	  if (MeasurementPackage::LASER == meas_package.sensor_type_) {
	  	//px - measured
	  	out_file_ <<meas_package.raw_measurements_(0) << "\t";
	  	//py - measured
	  	out_file_ << meas_package.raw_measurements_(1) << "\t";

	  } else if (MeasurementPackage::RADAR == meas_package.sensor_type_) {
	  	// measured values in polar coordinates
	  	float ro = meas_package.raw_measurements_(0);
	    float theta = meas_package.raw_measurements_(1);
	    //to cartesian coordinates
	    out_file_ << ro *cos(theta) << "\t"; //px - measured
	    out_file_ << ro *sin(theta) << "\t"; //py - measured
	  }

	  // output the ground truth values
	  out_file_ << gt_values(0) << "\t";
	  out_file_ << gt_values(1) << "\t";
	  out_file_ << gt_values(2) << "\t";
	  out_file_ << gt_values(3) << "\t";


	  //output NIS value
	  if (MeasurementPackage::LASER == meas_package.sensor_type_) {
	  	out_file_ << ukf.NIS_lidar_ << "\t";
	  	out_file_ << meas_package.sensor_type_ << "\n";

	  } else if (MeasurementPackage::RADAR == meas_package.sensor_type_) {
	  	out_file_ << ukf.NIS_radar_ << "\t";
	  	out_file_ << meas_package.sensor_type_ << "\n";

		}
	}

}