#pragma once

#include <cmath>
#include <iostream>
using namespace std;

class PIDImpl {
public:
  PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki)
      : _dt(dt), _max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0),
        _integral(0) {}

  ~PIDImpl() {}
  double calculate(double setpoint, double pv) {
    // Calculate error
    double error = setpoint - pv;
    // Proportional term
    double Pout = _Kp * error;
    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;
    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;
    // Calculate total output
    double output = Pout + Iout + Dout;
    // Restrict to max/min
    if (output > _max)
      output = _max;
    else if (output < _min)
      output = _min;
    // Save error to previous error
    _pre_error = error;
    return output;
  }

private:
  double _dt;
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  double _pre_error;
  double _integral;
};

class PID {
public:
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  PID(double dt, double max, double min, double Kp, double Kd, double Ki) {
    pimpl = new PIDImpl(dt, max, min, Kp, Kd, Ki);
  };
  // Returns the manipulated variable given a setpoint and current process value
  double calculate(double setpoint, double pv) {
    return pimpl->calculate(setpoint, pv);
  }
  ~PID() { delete pimpl; }

private:
  PIDImpl *pimpl;
};