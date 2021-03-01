#ifndef PID_HPP
#define PID_HPP

struct pid_impl_s;
class pid_s {
public:
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  pid_s(double, double, double, double, double, double);
  double calc(double, double);
  ~pid_s();

private:
  struct pid_impl_s *pimpl;
};

#endif /* PID_HPP */
