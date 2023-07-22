#ifndef __LIFT_H__
#define __LIFT_H__

#include "motor.h"
#include "pid.h"

typedef struct lift *lift_t;

struct lift
{
    double target_speed = 0;
    struct pid motor_pid[2];
    struct motor_device motor[2];
};

#endif