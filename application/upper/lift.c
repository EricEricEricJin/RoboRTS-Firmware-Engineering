#include "lift.h"
#include "log.h"

#define MAX_RND 20

int32_t lift_cascade_init(struct lift *lift, const char *name,
                          struct pid_param inter_param, struct pid_param outer_param,
                          enum device_can can)
{
    lift->target_position = 0;
    // device name of two motors
    char motor_name[2][OBJECT_NAME_MAX_LEN] = {0};
    uint8_t name_len;
    int32_t err;
    name_len = strlen(name);
    memcpy(&motor_name[0][name_len], "_L\0", 3);
    memcpy(&motor_name[1][name_len], "_R\0", 3);

    // assign property of two motors
    for (int i = 0; i < 2; i++)
    {
        memcpy(&motor_name[i], name, name_len);
        lift->motor[i].can_periph = can;
        lift->motor[i].can_id = 0x205 + i; // 0x205 and 0x206
        lift->motor[i].init_offset_f = 1;
        
        pid_struct_init(&lift->outer_pid[i], outer_param.max_out, outer_param.integral_limit, outer_param.p, outer_param.i, outer_param.d);    
        pid_struct_init(&(lift->inter_pid[i]), inter_param.max_out, inter_param.integral_limit, inter_param.p, inter_param.i, inter_param.d);
        
        err = motor_register(&(lift->motor[i]), motor_name[i]);
        if (err != E_OK)
        {
            goto end;
        }
    }

    return E_OK;
end:
    return err;
}

int32_t lift_cascade_calculate(struct lift *lift)
{
    device_assert(lift != NULL);

    float outer_out; // target speed, output by outer PID
    float motor_out; // current sent to motor
    float sensor_rnd, sensor_rpm;
    struct motor_data* pdata;
    float target_position;

    for (int i = 0; i < 2; i++)
    {
        pdata = motor_get_data(&(lift->motor[i]));
        sensor_rnd = (pdata->total_angle / 360.0f); // unit: round
        sensor_rpm = (pdata->speed_rpm);

        // log_i("rnd%d=%.1f", i, sensor_rnd);

        if (i == 0)
            target_position = lift->target_position;
        else
            target_position = -(lift->target_position);
        
        outer_out = pid_calculate(&(lift->outer_pid[i]), sensor_rnd, target_position);
        motor_out = pid_calculate(&(lift->inter_pid[i]), sensor_rpm, outer_out);
        motor_set_current(&(lift->motor[i]), (int16_t)(motor_out));
    }

    return E_OK;
}


int32_t lift_set_position(struct lift* lift, float position)
{
    device_assert(lift != NULL);
    if (position < 0 || position >= MAX_RND)
        return E_OK;
    lift->target_position = position;
    return E_OK;
}

int32_t lift_set_delta(struct lift* lift, float delta)
{
    device_assert(lift != NULL);
    lift->target_position += delta;
    if (lift->target_position < 0)
        lift->target_position = 0;
    else if (lift->target_position > MAX_RND)
        lift->target_position = MAX_RND;
    return E_OK;
}