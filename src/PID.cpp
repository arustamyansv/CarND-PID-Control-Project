#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // set coefficients
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  // init errors as 0
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  d_error = (cte - p_error);
  p_error = cte;
  i_error += p_error;
}

double PID::TotalError() {
  return Kp*p_error + Ki*i_error + Kd*d_error;
}

