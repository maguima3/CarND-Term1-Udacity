#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(int N_i, int N_d) {
  this->N_i = N_i;
  this->N_d = N_d;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = i_error = d_error = 0.0;
  i_error_history.push_back(0.0);
  d_error_history.push_back(0.0);

}

void PID::UpdateError(double cte) {
  //reinit accumulative error values
  d_error = 0.0;
  i_error = 0.0;


  double prev_error = p_error;
  double rate_of_change = cte - prev_error;

  p_error = cte;

  d_error_history.push_back(rate_of_change);
  //accumulate d_error in d_error_history vector
  if (d_error_history.size() > N_d) {
    d_error_history.erase(d_error_history.begin());
  }
  for (double &n : d_error_history) {
    d_error += n;
  }


  i_error_history.push_back(cte);
  //accumulate i_error in i_error_history vector
  if (i_error_history.size() > N_i) {
    i_error_history.erase(i_error_history.begin());
  }
  //total i_error value
  for (double &n : i_error_history) {
    i_error += n;
  }
}

double PID::TotalError() {}

