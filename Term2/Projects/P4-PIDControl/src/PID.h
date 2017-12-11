#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp; //proportional
  double Ki; //integral
  double Kd; //derivative

  /*
   * Integral parameters
   */
  int N_i; //integral time
  std::vector<double> i_error_history;

  /*
   * Derivative parameters
   */
  int N_d; //derivative time
  std::vector<double> d_error_history;



  /*
  * Constructor
  */
  PID(int N_i, int N_d);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

};

#endif /* PID_H */
