#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define eps 0.001 //epsilon

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  is_initialized_ = false; // not initialized yet...

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Set weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {
    weights_(i) = 0.5 / (lambda_+n_aug_);
  }

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  
  // initial predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // initial augmented sigma points matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/8;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  //Initial NIS radar value
  NIS_radar_ = 0.0;

  //Initial NIS lidar value
  NIS_lidar_ = 0.0;

}

UKF::~UKF() {}

void UKF::Initialization(MeasurementPackage meas_package) {

  if (MeasurementPackage::LASER == meas_package.sensor_type_ && use_laser_) {
    //define variables for better readability
    double px = meas_package.raw_measurements_(0);
    double py = meas_package.raw_measurements_(1);
    //initial state values
    x_ << px, py, 4, 0.5, 0;

    //initialize state covariance matrix
	  P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
	        0, std_laspy_*std_laspy_, 0, 0, 0,
	        0, 0, 1, 0, 0,
	        0, 0, 0, 1, 0,
	        0, 0, 0, 0, 1;
  }
  if (MeasurementPackage::RADAR == meas_package.sensor_type_ && use_radar_) {
    //define variables for better readability
    double ro = meas_package.raw_measurements_(0);
    double theta = meas_package.raw_measurements_(1);
    double ro_dot = meas_package.raw_measurements_(2);
    //polar equations
    double px = ro * cos(theta);
    double py = ro * sin(theta);
    //initial state values
    x_ << px, py, 4, ro_dot * cos(theta), ro_dot * sin(theta);

    //initialize state covariance matrix
	  P_ << std_radr_*std_radr_, 0, 0, 0, 0,
	        0, std_radr_*std_radr_, 0, 0, 0,
	        0, 0, 1, 0, 0,
	        0, 0, 0, std_radphi_, 0,
	        0, 0, 0, 0, std_radphi_;
  }

  //Initialize predicted sigma points
  Xsig_pred_.fill(0.0);

  is_initialized_ = true;

  //print initialization results
  cout << "---- EKF initialization ----" << endl;
  cout << "x_ = " << endl;
  cout << x_ << endl;
  cout << "P_ = "  << endl;
  cout << P_ << endl;
  cout << "----------------" << endl;


  //save initial timestamp for delta_t calculation (us)
  time_us_ = meas_package.timestamp_;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // skip predict/update if sensor type is ignored
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) ||
      (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_)) {
    return;
  }
  
  if (!is_initialized_) {
    Initialization(meas_package);
    return;

  }

  //compute the time elapsed between the current and previous measurements
  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0; //delta_t expressed in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  if(MeasurementPackage::RADAR == meas_package.sensor_type_ && use_radar_) {
    UpdateRadar(meas_package);
  }

  if (MeasurementPackage::LASER == meas_package.sensor_type_ && use_laser_) {
    UpdateLidar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  AugmentedSigmaPointsGeneration();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();
  
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //Extract measurement
  VectorXd z = meas_package.raw_measurements_;

  //Define measurement prediction and covariance
  VectorXd z_pred = VectorXd(3);
  MatrixXd S_pred = MatrixXd(3, 3);

  //define matrix for sigma points in radar measurement space
  MatrixXd Zsig = MatrixXd(3, 2*n_aug_+1);

  //Calculate measurement mean, covariance and transformed sigm apoints in radar space 
  PredictRadarMeasurement(&z_pred, &S_pred, &Zsig);

  //Update state
  UpdateStateRadar(z_pred, S_pred, Zsig, z);

  //Calculate NIS
  VectorXd diff = z - z_pred;
  NIS_radar_ = diff.transpose() * S_pred.inverse() * diff;

  // cout << "z: " << endl << z << endl;
  // cout << "x_: " << endl << x_ << endl;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  //Extract measurement
  VectorXd z = meas_package.raw_measurements_;

	//Define lidar measurement dimension
	int n_z = 2;

	//Define measurement prediction and covariance
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S_pred = MatrixXd(n_z, n_z);

  //define matrix for sigma points in radar measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

  //Calculate measurement mean, covariance and transformed sigm apoints in radar space 
  PredictLidarMeasurement(&z_pred, &S_pred, &Zsig);

  //Update state
  UpdateStateLidar(z_pred, S_pred, Zsig, z);

  //Calculate NIS
  VectorXd diff = z - z_pred;
  NIS_lidar_ = diff.transpose() * S_pred.inverse() * diff;

  // cout << "z: " << endl << z << endl;
  // cout << "x_: " << endl << x_ << endl;

}


/**
 * 
 */
void UKF::AugmentedSigmaPointsGeneration() {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  //create augmented state covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();

  //calculate augmented sigma points
  Xsig_aug_.col(0) = x_aug;
  for (int i=0; i<n_aug_; i++) {
    Xsig_aug_.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * A_aug.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A_aug.col(i);
  }

}

void UKF::SigmaPointPrediction(double delta_t) {

  //predict sigma points according to CTRV motion model
  for (int i=0; i<2*n_aug_+1; i++) {

    //define variables for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //define predicted state values
    double px_pred, py_pred;

    //avoid division by zero
    if (fabs(yawd) > eps) {
      px_pred = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_pred = p_y + v/yawd * (-cos(yaw + yawd*delta_t) + cos(yaw));

    } else {
      px_pred = p_x + v * cos(yawd) * delta_t;
      py_pred = p_y + v * sin(yawd) * delta_t;
    }

    double v_pred = v;
    double yaw_pred = yaw + yawd*delta_t;
    double yawd_pred = yawd;

    //add noise
    px_pred = px_pred + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_pred = py_pred + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_pred = v_pred + nu_a*delta_t;

    yaw_pred = yaw_pred + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_pred = yawd_pred + nu_yawdd*delta_t;

    //write predicted sigma points into right column
    Xsig_pred_(0, i) = px_pred;
    Xsig_pred_(1, i) = py_pred;
    Xsig_pred_(2, i) = v_pred;
    Xsig_pred_(3, i) = yaw_pred;
    Xsig_pred_(4, i) = yawd_pred;

  }
}

void UKF::PredictMeanAndCovariance() {

  //predict state mean
  x_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) { //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix
  P_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) { //iterate over sigma points

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

}



void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

  //define radar measurement space dimension
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(3);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  //transform sigma points into measurement space
  for (int i=0; i<2*n_aug_+1; i++) {
    //define variables for better readability
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    //transformation to radar space
    double rho = sqrt(p_x*p_x + p_y*p_y);
    double phi = atan2(p_y, p_x);
    double rho_dot = (1/rho) * (p_x*cos(yaw)*v + p_y*sin(yaw)*v);

    Zsig(0, i) = rho;
    Zsig(1, i) = phi;
    Zsig(2, i) = rho_dot;
  }

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //calculate measurement noise covariance
  MatrixXd R = MatrixXd(n_z, n_z);
  R.fill(0.0);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;

  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  S = S + R;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;

}

void UKF::UpdateStateRadar(VectorXd z_pred, MatrixXd S, MatrixXd Zsig, VectorXd z) {

  //define radar measurement space dimension
  int n_z = 3;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0);
  for (int i=0; i<2*n_aug_+1; i++) {
    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix
  VectorXd z_diff = z - z_pred; //residual
  while (z_diff(1)>M_PI) z_diff(1)-=2.*M_PI; //angle normalization
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}


void UKF::PredictLidarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

	//define lidar space dimension
	int n_z = 2;

	//create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

	for (int i=0; i<2*n_aug_+1; i++) {
    //define variables for better readability
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);

    //lidar measurement model 
    Zsig(0, i) = px;
    Zsig(1, i) = py;
  }

  //predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //calculate lidar measurement noise covariance
  MatrixXd R = MatrixXd(n_z, n_z);
  R.fill(0.0);
  R(0, 0) = std_laspx_ * std_laspx_;
  R(1, 1) = std_laspy_ * std_laspy_;

  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  S = S + R;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;

}


void UKF::UpdateStateLidar(VectorXd z_pred, MatrixXd S, MatrixXd Zsig, VectorXd z) {

	//define lidar space dimension
	int n_z = 2;

	//calculate cross-correlation matrix between sigma points in state space and measurement space
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  for (int i=0; i<2*n_aug_+1; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    while (x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //kalman gain
  MatrixXd K = Tc * S.inverse();

  x_ = x_ + K * (z - z_pred);
  P_ = P_ - K * S * K.transpose();

}
