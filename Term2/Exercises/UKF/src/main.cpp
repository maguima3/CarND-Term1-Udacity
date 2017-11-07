#include <iostream>
#include "Eigen/Dense"
#include <vector>
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

int main() {

	//Create a UKF instance
	UKF ukf;

/*******************************************************************************
* Programming assignment calls
*******************************************************************************/

  VectorXd x = VectorXd(5);
  MatrixXd P = MatrixXd(5, 5);
  ukf.UpdateState(&x, &P);

	return 0;
}