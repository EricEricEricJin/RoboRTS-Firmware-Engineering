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

#ifndef __ENGINEERING_H__
#define __ENGINEERING_H__

#include "sys.h"

#define FIRMWARE_VERSION_0 1u
#define FIRMWARE_VERSION_1 0u
#define FIRMWARE_VERSION_2 0u
#define FIRMWARE_VERSION_3 3u

#define FIRMWARE_VERSION ((FIRMWARE_VERSION_3 << 24) | (FIRMWARE_VERSION_2 << 16) | (FIRMWARE_VERSION_1 << 8) | FIRMWARE_VERSION_0)

#define MANIFOLD2_ADDRESS      0x00
#define CHASSIS_ADDRESS        0x01

/* cmd id can not be set 0xffff */
#pragma pack(push,1)

#define CMD_MANIFOLD2_HEART                 (0x0001u)
#define CMD_REPORT_VERSION                  (0x0002u)

#define CMD_STUDENT_DATA                    (0x0101u)

#define CMD_PUSH_CHASSIS_INFO               (0x0201u)
struct cmd_chassis_info
{
    int16_t gyro_angle;
    int16_t gyro_palstance;
    int32_t position_x_mm;
    int32_t position_y_mm;
    int16_t angle_deg;
    int16_t v_x_mm;
    int16_t v_y_mm;
};

#define CMD_SET_CHASSIS_MODE                (0x0202u)
#define CMD_SET_CHASSIS_SPEED               (0x0203u)
struct cmd_chassis_speed
{
    int16_t vx; // forward/back
    int16_t vy; // left/right
    int16_t vw; // anticlockwise/clockwise
    int16_t rotate_x_offset;
    int16_t rotate_y_offset;
};

#define CMD_GET_CHASSIS_PARAM               (0x0204u)
struct cmd_chassis_param
{
    uint16_t wheel_perimeter; /* the perimeter(mm) of wheel */
    uint16_t wheel_track;     /* wheel track distance(mm) */
    uint16_t wheel_base;      /* wheelbase distance(mm) */
    int16_t rotate_x_offset;
    int16_t rotate_y_offset;
};

#define CMD_SET_CHASSIS_SPD_ACC             (0x0205u)
struct cmd_chassis_spd_acc
{
    int16_t   vx;
    int16_t   vy;
    int16_t   vw;

    int16_t   ax;
    int16_t   ay;
    int16_t   wz;

    int16_t rotate_x_offset;
    int16_t rotate_y_offset;
};


#define CMD_SET_FRICTION_SPEED              (0x0304u)
struct cmd_firction_speed
{
    uint16_t left;
    uint16_t right;
};

#define CMD_RC_DATA_FORWORD                 (0x0401u)
#define CMD_PUSH_UWB_INFO                   (0x0402u)

#pragma pack(pop)

#endif // __ENGINEERING_H__
