
### Tasks
- System Tasks
   
| File | Task function | Thread ID |
|---| ---| ---|
|`Core/Src/freertos.c`|`services_task`|`SystemServicesHandle`|
|`components/soft_timer/os_timer.c`|`timer_task`|`timer_task_t`|
|`application/shell.c`|`pthread_cli`|`shell_task_t`|
|`application/sensor_task.c`|`sensor_task`|`sensor_task_t`|
|`application/offline_service.c`|`offline_service`|`offline_service_task_t`|
|`application/communicate.c`|`communicate_task ` | `communicate_task_t `

- User Tasks (chassis)

| File | Task function | Thread ID |
|---| ---| ---|
| `application/chassis/chassis_app.c` | `chassis_task` | `chassis_task_t` |

- User Tasks (gimbal)

| File | Task function | Thread ID |
|---| ---| ---|
| `application/gimbal/gimbal_app.c`| `gimbal_task`|`gimbal_task_t` |


### Chassis

#### Chassis Init

- Receive Command Table: Call callback function when receive the specified command;
    - Commands defined in `application/infantry_cmd.h`
    - Callback functions defined in `application/chassis_cmd.c`
    - Example: `int32_t student_data_transmit(uint8_t *buff, uint16_t len)`

```c
struct protocol_recv_cmd_obj chassis_recv_cmd_table[] =
{
    /* CMD | callback function */
    {CMD_STUDENT_DATA, student_data_transmit},
    {CMD_PUSH_GIMBAL_INFO, follow_angle_info_rcv},
    {CMD_SET_CHASSIS_SPEED, chassis_speed_ctrl},
    {CMD_SET_CHASSIS_SPD_ACC, chassis_spd_acc_ctrl},
    {CMD_MANIFOLD2_HEART, chassis_manifold_heart},
};
```

- Offline Table: handle behaviors when detect module offline

```c
struct offline_obj chassis_offline_table[] =
{
    /* event | enable | beep_times | offline time | offline_first_func| offline_func | online_first_func | online_func */
    {SYSTEM_PROTECT, ENABLE, 0, 0, 0, 0, chassis_offline, chassis_online, NULL},

    {OFFLINE_DBUS, ENABLE, OFFLINE_ERROR_LEVEL, 0, 100, NULL, NULL, NULL, NULL},

    {OFFLINE_CHASSIS_MOTOR1, ENABLE, OFFLINE_ERROR_LEVEL, 1, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_CHASSIS_MOTOR2, ENABLE, OFFLINE_ERROR_LEVEL, 2, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_CHASSIS_MOTOR3, ENABLE, OFFLINE_ERROR_LEVEL, 3, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_CHASSIS_MOTOR4, ENABLE, OFFLINE_ERROR_LEVEL, 4, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_MANIFOLD2_HEART, DISABLE, OFFLINE_WARNING_LEVEL, BEEP_DISABLE, 700, NULL, chassis_heart_offline, NULL, chassis_heart_online},
    {OFFLINE_CONTROL_CMD, ENABLE, OFFLINE_WARNING_LEVEL, BEEP_DISABLE, 700, NULL, chassis_control_offline, NULL, chassis_control_online},

    {OFFLINE_GIMBAL_INFO, ENABLE, APP_PROTECT_LEVEL, 0, 700, NULL, gimbal_info_offline, NULL, gimbal_info_online},
};
```

- `chassis_app_init`: just copy this,,, too complicated
    - TODO: Figure out how to use app manager and event manager

```c
struct app_manage *app;
chassis_t p_chassis;

/*Send command and data(?) to gimbal*/
protocol_can_interface_register("can1_0x500_to_0x600", 1024, 1, CAN1_PORT, GIMBAL_CAN_ID, CHASSIS_CAN_ID, can1_std_transmit);

app = get_current_app();
p_chassis = get_chassis();

app->local_addr = CHASSIS_ADDRESS;
app->recv_cmd_table = chassis_recv_cmd_table;
app->recv_cmd_tab_size = sizeof(chassis_recv_cmd_table) / sizeof(struct protocol_recv_cmd_obj);

app->send_cfg_table = NULL;

app->offline_table = chassis_offline_table;
app->offline_tab_size = sizeof(chassis_offline_table) / sizeof(struct offline_obj);

app->route_table = chassis_route_table;
app->route_tab_size = sizeof(chassis_route_table) / sizeof(struct route_obj);

app->can2_msg_callback = chassis_can2_callback;
app->dbus_rx_complete = chassis_dbus_rx_complete;

app->user_input_callback = chassis_input_handle;
app->user_key_callback = chassis_user_key_handle;

app->referee_cmd_callback = referee_data_send2pc;

soft_timer_register(chassis_info_push, p_chassis, 10);

osThreadDef(CHASSIS_TASK, chassis_task, osPriorityNormal, 0, 512);
chassis_task_t = osThreadCreate(osThread(CHASSIS_TASK), NULL);
```

#### Chassis Task

- Initialize remote control and event manager

```c
rc_info_t p_rc_info;

subscriber_t listSubs;
subscriber_t nolistSubs;

EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, chassis_dr16_data_update);

EventSubscribeInit(&nolistSubs, SUBS_MODE_NOLIST);
EventSubscribe(&nolistSubs, AHRS_MSG, AHRS_MSG_LEN, 0, NULL);

rc_device_register(&chassis_rc, "Chassis RC");
p_rc_info = rc_device_get_info(&chassis_rc);

```

- Initialize PID and soft timer for that

```c
chassis_pid_init(&chassis, "Chassis", chassis_motor_param, DEVICE_CAN2);

soft_timer_register((soft_timer_callback)chassis_pid_calculate, (void *)&chassis, 5);
soft_timer_register((soft_timer_callback)chassis_angle_broadcast, (void *)NULL, 10);

pid_struct_init(&pid_follow, MAX_CHASSIS_VW_SPEED, 50, 8.0f, 0.0f, 2.0f);

```
NOTE: Here the`MAX_CHASSIS_VW_SPEED` (max out of this PID) is defined in `components/algorithm/mecanum.h`

- Enter task loop

```c
/* dr16 data update */
EventMsgProcess(&listSubs, 0);
/* gyro data update */
EventMsgGetLast(&nolistSubs, AHRS_MSG, &chassis_gyro, NULL);

chassis_gyro_updata(&chassis, chassis_gyro.yaw * RAD_TO_DEG, chassis_gyro.gz * RAD_TO_DEG);
```

- Remote Controller
    - `rc_device_get_state(&chassis_rc, RC_S2_UP)`: get the switch state
    - `p_rc_info->ch2`: get the channel's input
