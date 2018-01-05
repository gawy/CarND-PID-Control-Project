#ifndef PID_H
#define PID_H

class PID {
private:
  long run_ctr;

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double run_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

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

  /** Return total run error . */
  double RunError();


private:

  double prev_cte = 0;
};

#endif /* PID_H */
