// PID.cpp
// PID error Controller with FeedForward library
#include "PID.h"

PID::PID() {
    Iterm_prev = 0;
    errorPrev = 0;
    Out_prev = 0;
    previousTime = millis();
}

float PID::calc(float error, float FFterm ) {
    unsigned long thisTime = millis();
    float dT = max(0.0001,(thisTime - previousTime));
    previousTime = thisTime;

    float Pterm = Kp * error;
    float Dterm = Kd * ( error - errorPrev ) / dT;
    float thisMax = constrain(Out_prev + rateLimInc*dT, Out_min, Out_max);
    float thisMin = constrain(Out_prev - rateLimInc*dT, Out_min, Out_max);
    float FPDterm = constrain(FFterm + Pterm + Dterm, thisMin, thisMax); // FeedForwardterm + Pterm + Dterm (limited to overall Min/Max and rate limited from previous OUT value)

    float Iterm = constrain((Ki * error * dT) + ( (1-(iForget*dT) ) * Iterm_prev ) , thisMin - FPDterm, thisMax - FPDterm); // staturate the Iterm term at man/max minus the other terms to avoid over-saturation of Iterm (aka Iterm or Ki "windup")
    float OUT =  FPDterm + Iterm;

    errorPrev = error;
    Iterm_prev = Iterm;
    Out_prev = OUT;

    if(echo){
        //Serial.print(" error = "); Serial.print(error);
        //Serial.print(" Kp = "); Serial.print(Kp);
        //Serial.print(" Pterm = "); Serial.print(Pterm);
        //Serial.print(" thisMin = "); Serial.print(thisMin);
        //Serial.print(" thisMax = "); Serial.print(thisMax);
        //Serial.print(" OUT = "); Serial.print(OUT);
    }

    return OUT;
}

void PID::setConstants(float Kp_, float Ki_, float Kd_, float iForget_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    iForget = iForget_;
    return;
}

void PID::setLimits(float Out_min_, float Out_max_) {
    Out_min = Out_min_;
    Out_max = Out_max_;
    return void();
}

void PID::setRateLimit(float rateLimInc_, float rateLimDec_) {
    rateLimInc = rateLimInc_;
    if (rateLimDec_==0){
      rateLimDec = rateLimInc_;
    } else {
      rateLimDec = rateLimDec_;
    }
    return void();
}

void PID::reset(float resetValue) {
    Iterm_prev = 0;
    errorPrev = resetValue;
    Out_prev = 0;
    previousTime = millis();
    return;
}
