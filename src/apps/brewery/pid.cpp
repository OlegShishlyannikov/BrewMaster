#include "apps/brewery/pid.hpp"

struct pid_impl_s {
public:
  pid_impl_s(double dt, double max, double min, double Kp, double Kd, double Ki);
  ~pid_impl_s();
  double calculate(double setpoint, double pv);

private:
  double dt_;
  double max_;
  double min_;
  double p_;
  double d_;
  double i_;
  double pre_error_;
  double integral_;
};

pid_s::pid_s(double dt, double max, double min, double p, double d, double i) { pimpl = new struct pid_impl_s(dt, max, min, p, d, i); }
double pid_s::calc(double setpoint, double pv) { return pimpl->calculate(setpoint, pv); }
pid_s::~pid_s() { delete pimpl; }

pid_impl_s::pid_impl_s(double dt, double max, double min, double p, double d, double i) : dt_(dt), max_(max), min_(min), p_(p), d_(d), i_(i), pre_error_(0), integral_(0) {}

double pid_impl_s::calculate(double setpoint, double pv) {

  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  double Pout = p_ * error;

  // Integral term
  integral_ += error * dt_;
  double Iout = i_ * integral_;

  // Derivative term
  double derivative = (error - pre_error_) / dt_;
  double Dout = d_ * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;

  // Restrict to max/min
  if (output > max_)
    output = max_;
  else if (output < min_)
    output = min_;

  // Save error to previous error
  pre_error_ = error;

  return output;
}

pid_impl_s::~pid_impl_s() {}
