#include "shared.h"
#include <math.h>

#define G 9.81f //meters per second squared
#define PI 3.14159f

void plant_step(vehicle_plant_t *plant, float dt){
    //simple kinematic model: just summing force members on the vehicle under test (obviously not very accurate, but easy to extend)
    //driving forces (newtons)
    float enginef = plant->maxf * plant->throttle;
    
    //resistive forces (newtons)
    float dragf = plant->drag_coeff * plant->speed * plant->speed; //drag coeff * speed squared ~ drag force
    float gravf = sinf(plant->slope * (PI / 180.0f)) * plant->mass * G; //horizontal component of gravitational force depending on slope angle
    float netf = enginef - dragf - gravf; //net horizontal forward force

    //F = ma
    plant->accel = netf / plant->mass;
    plant->speed += plant->accel * dt;

    //stop the vehicle (clamp speed and accel to zero) if it moves backwards (probably an error)
    if (plant->speed < 0.0f) {
        plant->speed = 0.0f;
        plant->accel = 0.0f;
    }
}
