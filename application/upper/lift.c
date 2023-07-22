#include "lift.h"

int32_t lift_pid_init(struct lift *lift, const char *name, struct pid_param param, enum device_can can)
{
    char motor_name[2][OBJECT_NAME_MAX_LEN] = {0};
    uint8_t name_len;
    int32_t err;

    name_len = strlen(name);
    for (int i = 0; i < 2; i++)
    {
        memcpy(&motor_name[i], name, name_len);
        lift->motor[i].can_periph = can;
        lift->motor[i].can_id + 0x205 + i; // 0x205 and 0x206
        lift->motor[i].init_offset_f = 1;

        pid_struct_init(&lift->motor_pid[i], param.max_out, param.integral_limit, param.p, param.i, param.d);
    }

    memcpy(&motor_name[0][name_len], "_L\0", 3);
    memcpy(&motor_name[1][name_len], "_R\0", 3);

    for (int i = 0; i < 2; i++)
    {
        err = motor_register(&(lift->motor[i]), motor_name[i]);
        if (err != E_OK)
        {
            goto end;
        }
        return E_OK;
    }

end:
    return err;
}

int32_t lift_pid_calculate(struct lift* lift)
{
    float motor_out;

    device_assert(lift != NULL);

    for (int i = 0; i < 2; i++)
    {
        motor_out = pid_calculate(&lift->motor_pid[i], lift->motor[i].data.speed_rpm, lift->target_speed_rpm);
        motor_set_current(&lift->motor[i], (int16_t)motor_out);
    }

    return E_OK;
}

int32_t lift_set_speed(struct lift* lift, double target_speed_rpm)
{
    if (lift == NULL)
    {
        return E_INVAL;
    }

    lift->target_speed_rpm = target_speed_rpm;

    return E_OK;
}

// int32_t lift_set_position()