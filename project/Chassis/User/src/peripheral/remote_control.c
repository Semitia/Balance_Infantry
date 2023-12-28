#include "remote_control.h"

RemoteController remote_controller;

void setControlModeAction(enum CONTROL_MODE_ACTION action)
{
    if (remote_controller.robot_state == CONTROL_MODE)
    {
        remote_controller.last_control_mode_action = remote_controller.control_mode_action;
        remote_controller.control_mode_action = action;
    }
}

void setGimbalAction(enum GIMBAL_ACTION action)
{
    remote_controller.last_gimbal_action = remote_controller.gimbal_action;
    remote_controller.gimbal_action = action;
}

void setShootAction(enum SHOOT_ACTION action)
{
    remote_controller.last_shoot_action = remote_controller.shoot_action;
    remote_controller.shoot_action = action;
}

void setLegAction(enum LEG_ACTION action)
{
    remote_controller.leg_action = action;
}

// 蓝牙
void Blue_Tooth_Deal(uint16_t *blue_tooth_recv)
{
    if (remote_controller.control_type == BLUE_TOOTH)
    {
        switch (*blue_tooth_recv)
        {
        case BLUE_TOOTH_OFFLINE_MODE:
            setRobotState(OFFLINE_MODE);
            setControlModeAction(NOT_CONTROL_MODE);
            remote_controller.blue_tooth_key = BLUE_TOOTH_NO_ACTION;
            break;
        case BLUE_TOOTH_CONTROL_MODE:
            setRobotState(CONTROL_MODE);
            setControlModeAction(BALANCE);
            remote_controller.blue_tooth_key = BLUE_TOOTH_NO_ACTION;
            break;
        case BLUE_TOOTH_ROTATE_MODE:
            setRobotState(CONTROL_MODE);
            setControlModeAction(SELF_ROTATE);
            remote_controller.blue_tooth_key = BLUE_TOOTH_NO_ACTION;
            break;
        // 机器人行为到任务程序处理
        case BLUE_TOOTH_UP:
            remote_controller.blue_tooth_key = BLUE_TOOTH_UP;
            break;
        case BLUE_TOOTH_LEFT:
            remote_controller.blue_tooth_key = BLUE_TOOTH_LEFT;
            break;
        case BLUE_TOOTH_STABLE:
            remote_controller.blue_tooth_key = BLUE_TOOTH_STABLE;
            break;
        case BLUE_TOOTH_RIGHT:
            remote_controller.blue_tooth_key = BLUE_TOOTH_RIGHT;
            break;
        case BLUE_TOOTH_DOWN:
            remote_controller.blue_tooth_key = BLUE_TOOTH_DOWN;
            break;
        default:
            break;
        }
    }
}

void RemoteLimit(unsigned short *ch, unsigned short limitValue)
{
    if ((*ch - CH_MIDDLE < limitValue) && (*ch - CH_MIDDLE > -limitValue))
        *ch = CH_MIDDLE;
}
void RemoteReceive(volatile unsigned char rx_buffer[])
{
    remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] = (rx_buffer[0] | (rx_buffer[1] << 8)) & 0x07ff;                              //!< Channel 0
    remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff;                       //!< Channel 1
    remote_controller.dji_remote.rc.ch[LEFT_CH_LR] = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff; //!< Channel 2
    remote_controller.dji_remote.rc.ch[LEFT_CH_UD] = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff;                        //!< Channel 3
    remote_controller.dji_remote.rc.s[RIGHT_SW] = ((rx_buffer[5] >> 4) & 0x0003);                                                 //!< Switch left
    remote_controller.dji_remote.rc.s[LEFT_SW] = ((rx_buffer[5] >> 6) & 0x0003);

    // 在底盘调试中，下面数据不起作用
    remote_controller.dji_remote.mouse.x = rx_buffer[6] | (rx_buffer[7] << 8);   //!< Mouse X axis
    remote_controller.dji_remote.mouse.y = rx_buffer[8] | (rx_buffer[9] << 8);   //!< Mouse Y axis
    remote_controller.dji_remote.mouse.z = rx_buffer[10] | (rx_buffer[11] << 8); //!< Mouse Z axis
    remote_controller.dji_remote.mouse.press_l = rx_buffer[12];                  //!< Mouse Left Is Press ?
    remote_controller.dji_remote.mouse.press_r = rx_buffer[13];                  //!< Mouse Right Is Press ?

    // 按键
    // remote_controller.dji_remote.last_keyValue = remote_controller.dji_remote.keyValue;
    memcpy(&remote_controller.dji_remote.keyValue, (void *)&rx_buffer[14], 2);
    // remote_controller.dji_remote.keyChangeOn = (remote_controller.dji_remote.last_keyValue ^ remote_controller.dji_remote.keyValue) &
    //                                            remote_controller.dji_remote.keyValue;
    // remote_controller.dji_remote.keyChangeOff = (remote_controller.dji_remote.last_keyValue ^ remote_controller.dji_remote.keyValue) &
    //                                             remote_controller.dji_remote.last_keyValue;
    // 16,17暂不使用

    for (int i = 0; i < 4; i++)
    {
        RemoteLimit(&remote_controller.dji_remote.rc.ch[i], 20);
    }

    // debug
    global_debugger.remote_debugger.dt = GetDeltaT(&global_debugger.remote_debugger.last_cnt);
    global_debugger.remote_debugger.recv_msgs_num++;
}

void RC_Rst(void)
{
    for (int i = 0; i < 4; i++)
    {
        remote_controller.dji_remote.rc.ch[i] = CH_MIDDLE;
    }
    remote_controller.dji_remote.mouse.x = 0;
    remote_controller.dji_remote.mouse.y = 0;
    remote_controller.dji_remote.mouse.z = 0;
    remote_controller.dji_remote.mouse.press_l = 0;
    remote_controller.dji_remote.mouse.press_r = 0;

    remote_controller.dji_remote.keyValue = 0;

    remote_controller.dji_remote.rc.s[LEFT_SW] = Down;
    remote_controller.dji_remote.rc.s[RIGHT_SW] = Down;
}

void setControlMode(enum CONTROL_TYPE type)
{
    remote_controller.control_type = type;
}

void setRobotState(enum ROBOT_STATE state)
{
    remote_controller.last_robot_state = remote_controller.robot_state;
    remote_controller.robot_state = state;
    if (remote_controller.robot_state != CONTROL_MODE) // 掉线模式
    {
        remote_controller.control_mode_action = NOT_CONTROL_MODE;
        setControlModeAction(NOT_CONTROL_MODE);
        setShootAction(SHOOT_POWERDOWN_MODE);
        setGimbalAction(GIMBAL_POWERDOWN);
        setLegAction(RAW_LENGTH);
    }
}

void setSuperPower(enum PowerControlState super_power_state)
{
    remote_controller.super_power_state = super_power_state;
}

void initRemoteControl(enum CONTROL_TYPE type)
{
    setControlMode(type);
    setRobotState(OFFLINE_MODE);

    if (type == DJI_REMOTE_CONTROL)
    {
        RC_Rst();
    }
}
