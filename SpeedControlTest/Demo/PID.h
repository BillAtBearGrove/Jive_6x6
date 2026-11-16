#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID
{
  public:
    PID();
    float calc(float error, float FFterm = 0);
    void setConstants(float Kp_, float Ki_, float Kd_, float iForget_ = 0); // setConstants(float Kp, float Ki, float Kd, optional float iForget = 0)
    void setLimits(float Out_min_, float Out_max_);
    void setRateLimit(float rateLimInc_, float rateLimDec_ = 0);
    void reset(float resetValue = 0);

  private:
    float Kp = 0.1;
    float Ki = 0;
    float Kd = 0;
    float iForget = 0;
    float Iterm_prev = 0;
    float errorPrev = 0;
    float Out_min = -1;
    float Out_max = 1;
    float Out_prev = 0;
    float rateLimInc = 10000; // effectively disabled as default
    float rateLimDec = 10000;
    unsigned long previousTime = millis(); // init at current time
};

#endif