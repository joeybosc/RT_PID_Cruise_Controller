#ifndef SHARED_H
#define SHARED_H

#include <stdint.h>
#include <stdatomic.h>

#define PI 3.14159f

#define SAMPLE_PERIOD_MS 10
#define DT_SEC 0.01f

//PID parameters struct
typedef struct {

    //accessible by frontend utility
    float kp;
    float ki;
    float kd;
    float setpoint; //target speed, in meters per second

    //internal use only
    float integral;
    float prev_error;
} pid_params_t;

//antiwindup limits for PID controller (can probably move this, add to TODO)
typedef struct {
    float integral_max;
    float integral_min;
} pid_limits_t;

//vehicle plant struct - modifiable parameters are mass, drag_coeff, maxf, and slope
typedef struct {
    float speed; //meters per second
    float accel; //meters per second squared

    float mass; //kg
    float drag_coeff; //defined experimentally, ratio of drag force to speed: N / ((m/s)^2)
    float slope; //degrees

    //Actuator members
    float throttle; //amount of juice from 0 to 1
    float maxf; //maximum force of engine
} vehicle_plant_t;

//plant model function prototypes
void plant_step(vehicle_plant_t *plant, float dt);

//containers and flags for parameter update process
extern volatile pid_params_t pid_staged_params;
extern atomic_uint_fast8_t pid_ready;

extern volatile vehicle_plant_t plant_staged_params;
extern atomic_uint_fast8_t plant_ready;

#endif 
