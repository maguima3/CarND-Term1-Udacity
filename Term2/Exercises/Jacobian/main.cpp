#include <iostream>
#include "Dense"
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {

	/*
	 * Compute the Jacobian Matrix
	 */

	//predicted state  example
	//px = 1, py = 2, vx = 0.2, vy = 0.4
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = CalculateJacobian(x_predicted);

	cout << "Hj:" << endl << Hj << endl;

	return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 

	//check division by zero
	if ( (px+py) <= 0.0001) {
		cout << "CalculateJacobian() - Error - Division by zero" << endl;
		return Hj;
	}
	
	//compute the Jacobian matrix
	// power of two
	float px2 = px * px;
	float py2 = py * py;
	//denominador to calculate rho, phi, and rhodot
	float den_rho = sqrt(px2 + py2);
	float den_phi = px2 + py2;
	float den_rhodot1 = pow(den_phi, 3/2);
	float den_rhodot2 = sqrt(den_phi);

	Hj << px/den_rho, py/den_rho, 0, 0,
		  -py/den_phi, px/den_phi, 0, 0,
		  py*(vx*py-vy*px)/den_rhodot1, px*(vy*px-vx*py)/sqrt(den_rhodot1), px/den_rhodot2, py/den_rhodot2;


	return Hj;
}