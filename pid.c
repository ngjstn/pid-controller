#include "pid.h"


// clear controller variables 
void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.0f; 
    pid->prevError = 0.0f; 

    pid->differentiator = 0.0f; 
    pid->prevMeasurement = 0.0f; 

    pid->out = 0.0f; 
}


// returns the controller output 
float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

    // error signal 
    float error = setpoint - measurement; 

    // proportional 
    float proportional = pid->Kp * error; 


    // compute integral and derivative terms
    calculateIntegral(pid, error, proportional); 
    calculateDerivative(pid, measurement); 


    // compute output (summation of all the terms) 
    pid->out = proportional + pid->integrator + pid->differentiator; 

    // apply user defined limits
    if (pid->out > pid->limMax) {
        pid->out = pid->limMax; 
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin; 
    }


    // store error and measurement for late use 
    pid->prevError = error; 
    pid->prevMeasurement = measurement; 

    // final controller output 
    return pid->out; 
}


// helper function for updating
void calculateIntegral(PIDController *pid, float error, float proportional) {
    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError); 


    // anti-wind-up via dynamic integrator clamping 
    float limMinInt, limMaxInt; 

    // compute integrator limits (to not saturate the output)
    if (pid->limMax > proportional) {
        limMaxInt = pid->limMax - proportional; 
    } else {
        limMaxInt = 0.0f; 
    }

    if (pid->limMin < proportional) {
        limMinInt = pid->limMin - proportional; 
    } else {
        limMinInt = 0.0f; 
    }

    // clamp integrator (actually limit the integrator value)
    if (pid->integrator > limMaxInt) {
        pid->integrator = limMaxInt; 
    } else if (pid->integrator < limMinInt) {
        pid->integrator = limMinInt; 
    }
}


// helper function for updating
void calculateDerivative(PIDController *pid, float measurement) {
    // derivative calculated on measurement, not on error signal 
    // avoids derivative "kick" during setpoint change 
    pid->differentiator = (2.0f * pid->Kd * (measurement - pid->prevMeasurement)  
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);  
}