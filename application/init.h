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

#ifndef __INIT_H__
#define __INIT_H__

#define JMP_DRIVER 0
#define NOJMP_DRIVER 1

#define JMP_PID 0
#define NOJMP_PID 1

#define JMP_SPEED 0
#define NOJMP_SPEED 1

#include "os_timer.h"

uint8_t get_driver_cfg(void);
uint8_t get_pid_cfg(void);
uint8_t get_speed_cfg(void);
void hw_init(void);
void task_init(void);

#endif // __INIT_H__
