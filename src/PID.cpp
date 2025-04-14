#include "PID.h"

PID::PID(float kp, float ki, float kd, int max_value, int min_value) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _last_error = 0;
    _i_error = 0;
    _max_value = max_value;
    _min_value = min_value;
}
PID::~PID() {
    // Destructor code (if needed)
}

void PID::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::setMaxMin(int max_value, int min_value) {
    _max_value = max_value;
    _min_value = min_value;
}

int PID::compute(int setpoint, int measurement) {
    // Calculate the error
    int error = setpoint - measurement;

    // Proportional term
    int p = _kp * error;

    // Integral term
    _i_error += error;
    int i = _ki * _i_error;

    // Derivative term

    int d = _kd * (error - _last_error);
    _last_error = error;

    // Calculate the output
    int output = p + i + d;

    // Clamp the output
    if (output > _max_value) {
        output = _max_value;
    } else if (output < _min_value) {
        output = _min_value;
    }
    return output;
}
