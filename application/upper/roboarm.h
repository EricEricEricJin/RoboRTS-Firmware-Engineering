#ifndef __ROBOARM_H__
#define __ROBOARM_H__

#include "motor.h"
#include "pid.h"

#define ROBOARM_PITCH_OFFSET (170)
#define ROBOARM_PITCH_MAX (60)
#define ROBOARM_PITCH_MIN (-179)

#define ROBOARM_PITCH_DEADZONE (110)

#define ROBOARM_ROLL_OFFSET (0)
#define ROBOARM_ROLL_MAX (90)
#define ROBOARM_ROLL_MIN (-90)

typedef struct roboarm *roboarm_t;

struct roboarm
{
    float pitch_target;
    float roll_target;

    struct pid pitch_outer_pid;
    struct pid pitch_inter_pid;
    struct pid roll_outer_pid;
    struct pid roll_inter_pid;

    struct motor_device pitch_motor[2];
    struct motor_device roll_motor;
};

int32_t roboarm_cascade_init(struct roboarm *roboarm, const char *name,
                             struct pid_param pitch_inter_param, struct pid_param pitch_outer_param,
                             struct pid_param roll_inter_param, struct pid_param roll_outer_param, enum device_can can);

int32_t roboarm_cascade_calculate(struct roboarm* roboarm);

int32_t roboarm_set_position(struct roboarm* roboarm, float pitch, float roll);
int32_t roboarm_set_delta(struct roboarm* roboarm, float delta_pitch, float delta_roll);

#endif