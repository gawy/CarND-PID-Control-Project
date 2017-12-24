#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

double prev_cte = 0;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;
}

void PID::UpdateError(double cte) {
  p_error = cte * Kp;

  d_error = (prev_cte - cte) * Kd;
  prev_cte = cte;

  i_error += cte * Ki;

  cout<<"Errors: p="<<p_error << ", d="<<d_error <<", i="<<i_error<<endl;
}

double PID::TotalError() {
  return p_error - d_error - i_error;
}

