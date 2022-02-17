#include <algorithm>
#include "pid.h"

PID::
PID(double* measured_signal, double* control_signal, double* set_point, const double Kp, const double Ki, const double Kd)
  : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_measured_signal(measured_signal), m_control_signal(control_signal), m_set_point(set_point), e_t_1(0.0), e_t_sum(0.0), m_lower_limit(-1000000000.0), m_upper_limit(1000000000.0), m_lower_threshold(0.000000001), m_upper_threshold(-0.000000001), m_lower_offset(0.000000001), m_upper_offset(-0.000000001){}

PID::
~PID(){}

void
PID::
compute(){
  double e_t = *m_set_point - *m_measured_signal;
  e_t_sum += e_t;
  *m_control_signal = (m_Kp * e_t) + (m_Ki * e_t_sum) + (m_Kd * (e_t - e_t_1));
  double sig_offset = 0;
  sig_offset = *m_control_signal >= m_upper_threshold? m_upper_offset: sig_offset;
  sig_offset = *m_control_signal <= m_lower_threshold? m_lower_offset: sig_offset;
  *m_control_signal += sig_offset;
  *m_control_signal = std::min(std::max(m_lower_limit, *m_control_signal), m_upper_limit);
  e_t_1 = e_t;
}

void 
PID::
set_signal_limits(const double lower_limit, const double upper_limit){
  m_lower_limit = lower_limit;
  m_upper_limit = upper_limit;
}

void
PID::
set_signal_offsets(const double lower_offset, const double upper_offset){
  m_lower_offset = lower_offset;
  m_upper_offset = upper_offset;
}


void
PID::
set_signal_thresholds(const double lower_threshold, const double upper_threshold){
  m_lower_threshold = lower_threshold;
  m_upper_threshold = upper_threshold;
}

