#ifndef __LIFT_H__
#define __LIFT_H__

#include "motor.h"
#include "pid.h"

typedef struct lift *lift_t;

struct lift
{
    float target_position; // unit: mm
    struct pid outer_pid;
    struct pid inter_pid[2];
    struct motor_device motor[2];
};

int32_t lift_cascade_init(struct lift *lift, const char *name,
                          struct pid_param inter_param, struct pid_param outer_param,
                          enum device_can can);

int32_t lift_cascade_calculate(struct lift *lift);

int32_t lift_set_position(struct lift* lift, float position);
int32_t lift_set_delta(struct lift* lift, float delta);

#endif