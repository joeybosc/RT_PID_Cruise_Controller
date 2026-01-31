/*
CONTROLTASK, COMPRISES:
            PARAMETER UPDATE FUNCTION (gets new parameters from shared memory via DMA) 
            CONTROL TASK (computes PID gain at each timestep)
            CONTROL TASK CREATOR
*/

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "shared.h"

#include "FreeRTOS.h"
#include "task.h"

#define CONTROL_TASK_STACK_SIZE 4096
#define CONTROL_TASK_PRIORITY (configMAX_PRIORITIES - 1)

#define PRINT_PERIOD_MS 1000

//instantiate plant and control objects and initialize default parameter sets
static vehicle_plant_t vehicle = {0.0f, 0.0f, 1000.0f, 0.5f, 0.0f, 0.0f, 2000.0f}; //init plant (vehicle): {speed, accel, mass, drag_coeff, slope, throttle, max_force}
static pid_params_t active_params = {10.0f, 1.0f, 0.1f, 20.0f, 0.0f, 0.0f}; //init pid params: {kp, ki, kd, setpoint, integral, prev_error}
static pid_limits_t limits = {100.0f, -100.0f}; //{integral max, integral min}

static uint64_t wcet_ns = 0;

//update the pid parameters with those loaded into the staging struct by the server
static void update_pid_params(void){

    //lock-free parameter copy
    uint8_t flag = atomic_load_explicit(&pid_ready, memory_order_acquire);
    
    if (flag) {
        //use atomic operation to copy new parameters into the active parameter instance (ensures no mutex/locking)
        active_params.kp = pid_staged_params.kp;
        active_params.ki = pid_staged_params.ki;
        active_params.kd = pid_staged_params.kd;
        active_params.setpoint = pid_staged_params.setpoint;

        //enforce reset of integral and error terms when parameters change
        active_params.integral = 0.0f;
        active_params.prev_error = 0.0f;

        atomic_store_explicit(&pid_ready, 0, memory_order_release);
        printf("Updated Control Parameters: Kp=%.3f, Ki=%.3f, Kd=%.3f, setpoint=%.3f m/s\n", active_params.kp, active_params.ki, active_params.kd, active_params.setpoint);
    }
}

//does the same thing, but for the plant parameters
static void update_plant_params(void){

    uint8_t flag = atomic_load_explicit(&plant_ready, memory_order_acquire);

    if (flag) {
        vehicle.mass = plant_staged_params.mass;
        vehicle.drag_coeff = plant_staged_params.drag_coeff;
        vehicle.maxf = plant_staged_params.maxf;
        vehicle.slope = plant_staged_params.slope;

        atomic_store_explicit(&plant_ready, 0, memory_order_release);
        printf("Updated Plant Parameters: mass=%.0f kg, drag=%.3f, engine force = %.0f N, slope = %.1f deg\n", vehicle.mass, vehicle.drag_coeff, vehicle.maxf, vehicle.slope * (PI / 180.0f));

    }
}

static uint64_t nanoseconds(struct timespec tv){
    return ((tv.tv_sec * 1000000000ULL) + tv.tv_nsec);
}

//the control task - does the PID math which could be modularized later
static void control_task(void *params) {
    //silence clang warning >:(
    (void)params;
    
    //when the task is woken by the scheduler, note the time
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_print_time = xTaskGetTickCount();

    int count = 0;

    printf("Control Task Started\n");
    printf("Vehicle Parameters: m = %.0f kg, engine force = %.0f N, drag coeff. = %.3f (N/(m/s)^2)\n", vehicle.mass, vehicle.maxf, vehicle.drag_coeff);
    printf("Environmental Parameters: road slope = %.0f\n", vehicle.slope);

    //main task loop: runs continously with deterministic blocking by higher priority tasks
    for(;;){

        struct timespec t0, t1; //timespec instead of timeval for POSIX call, nanosecond resolution!
        clock_gettime(CLOCK_MONOTONIC, &t0); //START TIMING EXECUTION HERE

        //updating here is very fast, but still possible to break determinism if deadline is exceeded by unexpected behavior
        update_pid_params();
        update_plant_params();

        float speedometer = vehicle.speed;

        //pid math with the shared memory object (no context switch required)
        float error = active_params.setpoint - speedometer;
        active_params.integral += error * DT_SEC;

        //clamp integral term to limits and prevent windup
        if (active_params.integral > limits.integral_max){active_params.integral = limits.integral_max;}
        if (active_params.integral < limits.integral_min){active_params.integral = limits.integral_min;}
        
        float derivative = (error - active_params.prev_error) / DT_SEC;

        float control_term = (active_params.kp * error) + (active_params.ki * active_params.integral) + (active_params.kd * derivative);

        active_params.prev_error = error;

        //scale control gain to the throttle bounds (0 to 1), and set the vehicle throttle accordingly.
        float throttle = control_term / 100.0f;

        //clamp the control signal to allowed range (shouldn't be necessary if antiwindup limits are set properly, but keep for robustness)
        if (throttle < -1.0f) {throttle = -1.0f;}
        if (throttle > 1.0f) {throttle = 1.0f;}

        //update throttle 
        vehicle.throttle = throttle;

        //step plant model forward by exactly one sample step
        plant_step(&vehicle, DT_SEC);
        
        //print the readout every one second real time
        TickType_t current_time = xTaskGetTickCount();
        if(current_time - last_print_time >= pdMS_TO_TICKS(PRINT_PERIOD_MS)){
            printf("Speed: %.1f m/s (target: %1f m/s) | Throttle: %.1f%% | Error: %.2f m/s\n", speedometer, active_params.setpoint, vehicle.throttle * 100.0f, error);
            last_print_time = current_time;
        }

        //END TIMING EXECUTION HERE
        clock_gettime(CLOCK_MONOTONIC, &t1);

        uint64_t elapsed_ns = nanoseconds(t1) - nanoseconds(t0);
        if (elapsed_ns > wcet_ns){
            wcet_ns = elapsed_ns;
            printf("Updated WCET to %lu after %d iterations\n", (unsigned long)wcet_ns, count);
        } //keep updating WCET
        count++; 

        //wait until next cycle
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

//control task creation: xTaskCreate takes (pxTaskCode, *pcName, usStackDepth, *pvParameters, uxPriority, *pxCreatedTask)
BaseType_t controlTask_create(void){
    return xTaskCreate(control_task, "Control", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, NULL);
}

