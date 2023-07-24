#include "sys.h"
#include "dbus.h"
#include "event_mgr.h"
#include "event.h"
#include "os_timer.h"

#include "upper_task.h"
#include "lift.h"
#include "log.h"

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

uint8_t upper_mode = NORMAL_MODE;

static void upper_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);

struct lift lift;

struct rc_device upper_rc;

void upper_task(void const *argument)
{
    // Subscribe receiver event
    subscriber_t listSubs;

    EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, upper_dr16_data_update);

    rc_device_register(&upper_rc, "Upper RC");
    rc_info_t p_rc_info = rc_device_get_info(&upper_rc);

    // lift_cascade_init()
    lift_cascade_init(&lift, "Lift", lift_inter_param, lift_outer_param, DEVICE_CAN2);

    float lift_delta;
    while (1)
    {
        // Update receiver msg
        EventMsgProcess(&listSubs, 0);
        switch (upper_mode)
        {
        case NORMAL_MODE:
            lift_delta = (p_rc_info->ch4) * (-0.0005f);
            log_i("Lift target: %d", (int)(lift.target_position));
            lift_set_delta(&lift, lift_delta);
            lift_cascade_calculate(&lift);
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