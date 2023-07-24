#include "lift.h"
#include "log.h"

#define LIFT_DEG2MM 10

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

    // assign property of two motors
    for (int i = 0; i < 2; i++)
    {
        memcpy(&motor_name[i], name, name_len);
        lift->motor[i].can_periph = can;
        lift->motor[i].can_id = 0x205 + i; // 0x205 and 0x206
        lift->motor[i].init_offset_f = 1;
    }
    memcpy(&motor_name[0][name_len], "_L\0", 3);
    memcpy(&motor_name[1][name_len], "_R\0", 3);

    // initialize inter and outer pid
    pid_struct_init(&lift->outer_pid, outer_param.max_out, outer_param.integral_limit, outer_param.p, outer_param.i, outer_param.d);    
    pid_struct_init(&lift->inter_pid, inter_param.max_out, inter_param.integral_limit, inter_param.p, inter_param.i, inter_param.d);

    // register two motors
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

int32_t lift_cascade_calculate(struct lift *lift)
{
    float outer_out; // target speed, output by outer PID
    float motor_out; // current sent to motor
    float sensor_rnd, sensor_rpm;
    struct motor_data* pdata;

    device_assert(lift != NULL);

    // Use motor[0] as input

    pdata = motor_get_data(&(lift->motor[0]));

    sensor_rnd = (pdata->total_angle / 360.0f); // unit: round
    sensor_rpm = (pdata->speed_rpm);

    outer_out = pid_calculate(&(lift->outer_pid), sensor_rnd, lift->target_position);
    motor_out = pid_calculate(&(lift->inter_pid), sensor_rpm, outer_out);

    log_i("rnd:%d,rpm:%d,outer:%d,motor:%d", (int)sensor_rnd, (int)sensor_rpm, (int)outer_out, (int)motor_out);

    // log_i("Lift motor out: %d", (int16_t)motor_out);
    motor_set_current(&(lift->motor[0]), (int16_t)motor_out);
    motor_set_current(&(lift->motor[1]), (int16_t)(-motor_out));

    return E_OK;
}


int32_t lift_set_position(struct lift* lift, float position)
{
    device_assert(lift != NULL);
    lift->target_position = position;
}

int32_t lift_set_delta(struct lift* lift, float delta)
{
    lift->target_position += delta;
}