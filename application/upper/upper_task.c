#include "sys.h"
#include "dbus.h"
#include "event_mgr.h"
#include "event.h"
#include "os_timer.h"
#include "log.h"
#include "tim.h"

#include "stepper.h"

#include "upper_task.h"
#include "lift.h"
#include "roboarm.h"

struct pid_param lift_outer_param =
    {
        .p = 1200.0f,
        .max_out = 6000.0f};

struct pid_param lift_inter_param =
    {
        .p = 6.5f,
        .i = 0.1f,
        .max_out = 30000,
        .integral_limit = 500,
};

struct pid_param roboarm_roll_outer_param =
    {
        .p = 40.0f,
        .max_out = 2000.0f,
};

struct pid_param roboarm_roll_inter_param =
    {
        .p = 6.0f,
        .i = 0.1f,
        .max_out = 30000,
        .integral_limit = 3000,
};

struct pid_param roboarm_pitch_outer_param =
    {
        .p = 300.0f,
        .max_out = 3000,
};

struct pid_param roboarm_pitch_inter_param =
    {
        .p = 6.0f,
        .i = 0.1f,
        .max_out = 30000,
        .integral_limit = 3000,
};

uint8_t upper_mode = NORMAL_MODE;

static void upper_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);

struct lift lift;
struct roboarm roboarm;

struct rc_device upper_rc;

void upper_task(void const *argument)
{
    // Subscribe receiver event
    subscriber_t listSubs;

    EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, upper_dr16_data_update);

    rc_device_register(&upper_rc, "Upper RC");
    rc_info_t p_rc_info = rc_device_get_info(&upper_rc);

    // Initialize components
    lift_cascade_init(&lift, "Lift", lift_inter_param, lift_outer_param, DEVICE_CAN2);
    roboarm_cascade_init(&roboarm, "Roboarm", roboarm_pitch_inter_param, roboarm_pitch_outer_param, roboarm_roll_inter_param, roboarm_roll_outer_param, DEVICE_CAN2);
    set_stepper_speed(0);

    float lift_delta;
    float roboarm_pitch_delta, roboarm_roll_delta;

    while (1)
    {
        // Update receiver msg
        EventMsgProcess(&listSubs, 0);
        switch (upper_mode)
        {
        case NORMAL_MODE:
            // lift control
            lift_delta = 0;
            // SHIFT: up, CTRL: down
            if (p_rc_info->kb.bit.SHIFT)
                lift_delta = 0.05;
            else if (p_rc_info->kb.bit.CTRL)
                lift_delta = -0.05;
            else
                lift_delta = -(p_rc_info->wheel / 10000.0f); 

            lift_set_delta(&lift, lift_delta);
            lift_cascade_calculate(&lift);

            // roboarm control
            // QE: roll, RF: pitch
            if (p_rc_info->kb.bit.Q)
                roboarm_roll_delta = -0.7;
            else if (p_rc_info->kb.bit.E)
                roboarm_roll_delta = 0.7;
            else
                roboarm_roll_delta = 0;
            
            log_i("delta=%.1f", roboarm_roll_delta);
            
            if (p_rc_info->kb.bit.R)
                roboarm_pitch_delta = 0.7;
            else if (p_rc_info->kb.bit.F)
                roboarm_pitch_delta = -0.7;
            else             
                roboarm_pitch_delta = (p_rc_info->ch4) * (0.001f);


            roboarm_set_delta(&roboarm, roboarm_pitch_delta, roboarm_roll_delta);
            roboarm_cascade_calculate(&roboarm);

            // stepper control
            int16_t stepper_speed = 0;
            if (p_rc_info->mouse.z)
                stepper_speed = p_rc_info->mouse.z;
            else if (rc_device_get_state(&upper_rc, RC_S1_UP) == E_OK)
                stepper_speed = 20;
            else if (rc_device_get_state(&upper_rc, RC_S1_DOWN) == E_OK)
                stepper_speed = -20;
            
            VAL_LIMIT(stepper_speed, -20, 20);
            set_stepper_speed(stepper_speed);

            // pump control
            // mouse left
            if (p_rc_info->mouse.l)
                HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET);
            else if (p_rc_info->mouse.r)
                HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);
            else if (rc_device_get_state(&upper_rc, RC_S2_MID2UP) == E_OK)
                HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET);
            else if (rc_device_get_state(&upper_rc, RC_S2_UP2MID) == E_OK)
                HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);

            break;
        default:
            break;
        }

        osDelay(5);
    }
}

static void upper_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
    rc_device_date_update(&upper_rc, pMsgData);
}