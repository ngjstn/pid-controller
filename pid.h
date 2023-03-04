#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H 

typedef struct {

    // controller gains 
    float Kp; 
    float Ki; 
    float Kd; 

    // derivative low-pass filter time constant 
    float tau; 

    // output limits
    float limMin; 
    float limMax; 

    // sample time (in seconds) 
    float T; 

    // controller variables 
    float integrator; 
    float prevError;
    float differentiator; 
    float prevMeasurement; 

    // controller output 
    float out; 

} PIDController; 


void PIDController_Init(PIDController *pid); 
float PIDController_Update(PIDController *pid, float setpoint, float measurement); 

#endif 