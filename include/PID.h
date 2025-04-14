#ifndef PID_H_
#define PID_H_

class PID
{
private:
    float _kp;
    float _ki;
    float _kd;
    int _last_error;
    int _i_error;

    int _max_value;
    int _min_value;

public:
    PID(float, float, float, int, int);
    ~PID();
    
    void setGains(float kp, float ki, float kd);
    void setMaxMin(int max_value, int min_value);
    int compute(int setpoint, int measurement);

};

#endif