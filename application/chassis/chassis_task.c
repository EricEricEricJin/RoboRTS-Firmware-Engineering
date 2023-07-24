/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "dbus.h"
#include "chassis_task.h"
#include "chassis_cmd.h"
#include "os_timer.h"
#include "engineering_cmd.h"
#include "board.h"
#include "event_mgr.h"
#include "event.h"
#include "chassis.h"
#include "offline_service.h"
#include "init.h"

struct pid_param chassis_motor_param =
    {
        .p = 6.5f,
        .i = 0.1f,
        .max_out = 15000,
        .integral_limit = 500,
};

#define Y_MOVE_DZ 0

static void chassis_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);
// static int32_t chassis_angle_broadcast(void *argv);

struct chassis chassis;
struct rc_device chassis_rc;
struct ahrs_sensor chassis_gyro;

/* chassis speed */
static float vx, vy, wz;

/* fllow control */
struct pid pid_follow = {0};

void chassis_task(void const *argument)
{
    rc_info_t p_rc_info;

    subscriber_t listSubs;
    subscriber_t nolistSubs;

    EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, chassis_dr16_data_update);

    EventSubscribeInit(&nolistSubs, SUBS_MODE_NOLIST);
    EventSubscribe(&nolistSubs, AHRS_MSG, AHRS_MSG_LEN, 0, NULL);

    rc_device_register(&chassis_rc, "Chassis RC");
    p_rc_info = rc_device_get_info(&chassis_rc);

    chassis_pid_init(&chassis, "Chassis", chassis_motor_param, DEVICE_CAN2);

    soft_timer_register((soft_timer_callback)chassis_pid_calculate, (void *)&chassis, 5);
    // soft_timer_register((soft_timer_callback)chassis_angle_broadcast, (void *)NULL, 10);

    pid_struct_init(&pid_follow, MAX_CHASSIS_VW_SPEED, 50, 8.0f, 0.0f, 2.0f);

    float local_ch3;
    while (1)
    {
        /* dr16 data update */
        EventMsgProcess(&listSubs, 0);
        /* gyro data update */
        EventMsgGetLast(&nolistSubs, AHRS_MSG, &chassis_gyro, NULL);

        chassis_gyro_updata(&chassis, chassis_gyro.yaw * RAD_TO_DEG, chassis_gyro.gz * RAD_TO_DEG);

        // Chassis movement
        vx = (float)p_rc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
        vy = -(float)p_rc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
        wz = -(float)p_rc_info->ch3 / 660 * MAX_CHASSIS_VW_SPEED;
        chassis_set_offset(&chassis, 0, 0);
        chassis_set_acc(&chassis, 0, 0, 0);
        chassis_set_speed(&chassis, vx, vy, wz);

        // Lift


        osDelay(5);
    }
}

struct chassis *get_chassis(void)
{
    return &chassis;
}

/**
 * @brief  subscrib dr16 event, update
 * @param
 * @retval void
 */
static void chassis_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
    rc_device_date_update(&chassis_rc, pMsgData);
}