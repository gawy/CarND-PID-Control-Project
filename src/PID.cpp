#include "PID.h"
#include <iostream>

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;

  prev_cte = 0;
  p_error = 0;
  d_error = 0;
  i_error = 0;

  run_error = 0.0;
  run_ctr = 0;
}

void PID::UpdateError(double cte) {
  p_error = cte;

  d_error = (prev_cte - cte);
  prev_cte = cte;

  i_error += cte;

  run_error += cte*cte;
  run_ctr++;

//  cout<<"Errors: p="<<p_error << ", d="<<d_error <<", i="<<i_error<<endl;
}

double PID::TotalError() {
  return p_error * Kp - d_error * Kd - i_error * Ki;
}

double PID::RunError() {
  return run_error / run_ctr;
}

