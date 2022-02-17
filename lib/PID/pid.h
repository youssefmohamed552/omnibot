#ifndef PID_H
#define PID_H

class PID{
  private:
    const double m_Kp;
    const double m_Ki;
    const double m_Kd;
    double* m_measured_signal;
    double* m_control_signal;
    double* m_set_point;
    double e_t_1;
    double e_t_sum;
    double m_lower_limit;
    double m_upper_limit;
    double m_lower_threshold;
    double m_upper_threshold;
    double m_lower_offset;
    double m_upper_offset;


  public:
    PID(double* input, double* output, double* target, const double Kp, const double Ki, const double Kd);
    ~PID();
    void compute();
    void set_signal_limits(const double lower_limit, const double upper_limit);
    void set_signal_thresholds(const double lower_threshold, const double upper_threshold);
    void set_signal_offsets(const double lower_offset, const double upper_offset);

};

#endif
