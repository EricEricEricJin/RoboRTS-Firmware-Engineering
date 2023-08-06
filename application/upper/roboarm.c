#include "roboarm.h"
#include "log.h"
#include "my_math.h"

#define ANGLE_LIMIT_180_PM(val, angle) \
  do                                \
  {                                 \
    (val) = (angle) - (int)(angle); \
    (val) += (int)(angle) % 360;    \
    if ((val) > 180)                \
      (val) -= 360;                 \
    else if ((val) < -180)             \
        (val) += 360;               \
  } while (0)

int32_t roboarm_cascade_init(struct roboarm *roboarm, const char *name,
                             struct pid_param pitch_inter_param, struct pid_param pitch_outer_param,
                             struct pid_param roll_inter_param, struct pid_param roll_outer_param, enum device_can can)
{
    roboarm->roll_target = 0;
    roboarm->pitch_target = 0;

    char motor_name[3][OBJECT_NAME_MAX_LEN] = {0};
    uint8_t name_len;
    int32_t err;
    name_len = strlen(name);
    memcpy(&motor_name[0][name_len], "PITCH_L\0", 8);
    memcpy(&motor_name[1][name_len], "PITCH_R\0", 8);
    memcpy(&motor_name[2][name_len], "ROLL\0", 5);

    // initialize pitch motors; ID: 0x207, 0x209
    for (int i = 0; i < 2; i++)
    {
        memcpy(&motor_name[i], name, name_len);
        roboarm->pitch_motor[i].can_periph = can;
        roboarm->pitch_motor[i].can_id = 0x207 + i;
        roboarm->pitch_motor[i].init_offset_f = 1;

        err = motor_register(&(roboarm->pitch_motor[i]), motor_name[i]);
        if (err != E_OK)
            goto end;
    }

    // initialize pitch PID
    pid_struct_init(&(roboarm->pitch_outer_pid), pitch_outer_param.max_out, pitch_outer_param.integral_limit,
                    pitch_outer_param.p, pitch_outer_param.i, pitch_outer_param.d);
    pid_struct_init(&(roboarm->pitch_inter_pid), pitch_inter_param.max_out, pitch_inter_param.integral_limit,
                    pitch_inter_param.p, pitch_inter_param.i, pitch_inter_param.d);
    
    // initialize roll motor; ID: 0x208, 0x209
    memcpy(&motor_name[2], name, name_len);
    roboarm->roll_motor.can_periph = can;
    roboarm->roll_motor.can_id = 0x209;
    roboarm->roll_motor.init_offset_f = 1;

    err = motor_register(&(roboarm->roll_motor), motor_name[2]);
    if (err != E_OK)
        goto end;

    // initialize roll PID
    pid_struct_init(&(roboarm->roll_outer_pid), roll_outer_param.max_out, roll_outer_param.integral_limit,
                    roll_outer_param.p, roll_outer_param.i, roll_outer_param.d);
    pid_struct_init(&(roboarm->roll_inter_pid), roll_inter_param.max_out, roll_inter_param.integral_limit,
                    roll_inter_param.p, roll_inter_param.i, roll_inter_param.d);
    
    return E_OK;    

end:
    return err;
}


int32_t roboarm_cascade_calculate(struct roboarm* roboarm)
{
    device_assert(roboarm != NULL);

    float outer_out, motor_out, sensor_angle, sensor_rate; // unit: deg, deg/s
    struct motor_data* pdata;
    float target_angle;

    // pitch
    pdata = motor_get_data(&(roboarm->pitch_motor[0]));
    log_i("rawangle=%.1f", pdata->ecd / ENCODER_ANGLE_RATIO);
    ANGLE_LIMIT_180_PM(sensor_angle, pdata->ecd / ENCODER_ANGLE_RATIO - ROBOARM_PITCH_OFFSET);
    // range from -180 to 180 degree
    sensor_rate = pdata->speed_rpm * 6;
    target_angle = roboarm->pitch_target;

    // if (target_angle - sensor_angle > 180)
    //     sensor_angle += 360;
    // else if (target_angle - sensor_angle < -180)
    //     sensor_angle -=360 ;

    if (ROBOARM_PITCH_DEADZONE > VAL_MIN(sensor_angle, target_angle) && ROBOARM_PITCH_DEADZONE < VAL_MAX(sensor_angle, target_angle))
    {
        if (sensor_angle > target_angle)
            sensor_angle -= 360;
        else if (sensor_angle < target_angle)
            sensor_angle += 360;
    }

    outer_out = pid_calculate(&(roboarm->pitch_outer_pid), sensor_angle, target_angle);
    motor_out = pid_calculate(&(roboarm->pitch_inter_pid), sensor_rate, outer_out);
    motor_set_current(&(roboarm->pitch_motor[0]), (int16_t)(motor_out));
    motor_set_current(&(roboarm->pitch_motor[1]), (int16_t)(-motor_out));
    // log_i("Pitch Motor Out: %d", (int16_t)motor_out);

    // roll
    pdata = motor_get_data(&(roboarm->roll_motor));
    ANGLE_LIMIT_180_PM(sensor_angle, pdata->ecd / ENCODER_ANGLE_RATIO);
    sensor_rate = (pdata->ecd_raw_rate * 1000.0f / ENCODER_ANGLE_RATIO); // feedback frequency: 1000Hz
    target_angle = roboarm->roll_target;

    if (target_angle - sensor_angle > 180)
        sensor_angle += 360;
    else if (target_angle - sensor_angle < -180)
        sensor_angle -=360 ;

    outer_out = pid_calculate(&(roboarm->roll_outer_pid), sensor_angle, target_angle);
    motor_out = pid_calculate(&(roboarm->roll_inter_pid), sensor_rate, outer_out);
    motor_set_current(&(roboarm->roll_motor), (int16_t)(motor_out));

    // log_i("sensor=%.1f,target=%.1f,outer=%.1f", sensor_angle, target_angle, outer_out);

    return E_OK;
}

int32_t roboarm_set_position(struct roboarm* roboarm, float pitch, float roll)
{
    roboarm->pitch_target = pitch;
    VAL_LIMIT(roboarm->pitch_target, ROBOARM_PITCH_MIN, ROBOARM_PITCH_MAX);
    ANGLE_LIMIT_180_PM(roboarm->roll_target, roll);
    return E_OK;
}

int32_t roboarm_set_delta(struct roboarm* roboarm, float delta_pitch, float delta_roll)
{
    roboarm->pitch_target += delta_pitch;
    VAL_LIMIT(roboarm->pitch_target, ROBOARM_PITCH_MIN, ROBOARM_PITCH_MAX);
    ANGLE_LIMIT_180_PM(roboarm->roll_target, roboarm->roll_target + delta_roll);
    return E_OK;
}