"""
Controller of 4 HT_04 motrors, use one USB2CAN module.
"""

import sys
import signal
import os
import time
import signal
import numpy as np
from usb_2_can import USB2CAN

# UART config 
BOUND = 2000000
UART_DEV = "/dev/ttyUSB0"
# number of motors
LEFT = 0
RIGHT = 1
FRONT = 0
BACK = 1
# Motor CAN ID
LEFT_FRONT_HT_ID  = 0x60
LEFT_BACK_HT_ID   = 0x61
RIGHT_FRONT_HT_ID = 0x62
RIGHT_BACK_HT_ID  = 0x63
# Motor dict: id -> (left/right, front/back)
motor_dict = {
    LEFT_FRONT_HT_ID: (LEFT, FRONT),
    LEFT_BACK_HT_ID: (LEFT, BACK),
    RIGHT_FRONT_HT_ID: (RIGHT, FRONT),
    RIGHT_BACK_HT_ID: (RIGHT, BACK)
}
# Motor control command
CMD_MOTOR_MODE = 0x01
CMD_RESET_MODE = 0x02
CMD_ZERO_POSITION = 0x03
# Range of motor parameters
P_MIN = -95.5           # Radians
P_MAX = 95.5
V_MIN = -45.0           # Rad/s
V_MAX = 45.0
KP_MIN = 0.0            # N-m/rad
KP_MAX = 500.0
KD_MIN = 0.0            # N-m/rad/s
KD_MAX = 5.0
T_MIN = -18.0
T_MAX = 18.0

def limit(value, min_value, max_value):
    """
    限幅函数
    """
    return max(min(value, max_value), min_value)

def float_to_uint(x, x_min, x_max, bits):
    """
    浮点数转换为bits位无符号整数
    """
    span = x_max - x_min
    offset = x_min
    x = (x - offset) / span    # 将x映射到[0, 1]范围内
    x = x * (2**bits - 1)      # 将x映射到[0, 2^bits - 1]范围内
    return round(x)            # 四舍五入并转换为整数

def uint_to_float(x, x_min, x_max, bits):
    """
    bits位无符号整数转换为浮点数
    """
    span = x_max - x_min
    offset = x_min
    x = x / (2**bits - 1)      # 将x映射到[0, 1]范围内
    x = x * span + offset      # 将x映射到[x_min, x_max]范围内
    return x

class MotorState:
    def __init__(self):
        self.mode = 0           # 电机模式
        self.pos = 0            # 电机位置
        self.vel = 0            # 电机速度
        self.cur = 0            # 电机电流
        self.temp = 0           # 电机温度

class MotorCtrlCmd:
    def __init__(self):
        self.pos_tar = 0        # 位置控制目标
        self.vel_tar = 0        # 速度控制目标
        self.KP = 0             
        self.KD = 0
        self.cur_ff = 0         # 电流前馈

class HT_04:
    def __init__(self, id):
        self.id = id & 0xFF
        self.state = MotorState()
        self.ctrl_cmd = MotorCtrlCmd()
        
class HT_04_Ctrl:
    def __init__(self, port, baudrate):
        self.usb2can = USB2CAN(port, baudrate)
        self.usb2can.set_ATmode()
        self.motor =   [[HT_04(LEFT_FRONT_HT_ID),  HT_04(LEFT_BACK_HT_ID)], 
                        [HT_04(RIGHT_FRONT_HT_ID), HT_04(RIGHT_BACK_HT_ID)]]
        # 使能电机
        for i in range(2):
            for j in range(2):
                self.start_motor(self.motor[i][j])
        print("HT_04_Ctrl init ok!")
        
    def set_mode(self, motor, mode):
        msg = bytearray([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
        if mode == CMD_MOTOR_MODE:
            msg.append(0xFC)
        elif mode == CMD_RESET_MODE:
            msg.append(0xFD)
        elif mode == CMD_ZERO_POSITION:
            msg.append(0xFE)
        else:
            print("Error mode!")
            return
        self.usb2can.send_std(motor.id, msg)
        
    def send_para(self, motor, ctrl_cmd):
        # range limit
        pos_send = limit(ctrl_cmd.pos_tar, P_MIN, P_MAX)
        vel_send = limit(ctrl_cmd.vel_tar, V_MIN, V_MAX)
        KP_send = limit(ctrl_cmd.KP, KP_MIN, KP_MAX)
        KD_send = limit(ctrl_cmd.KD, KD_MIN, KD_MAX)
        cur_ff_send = limit(ctrl_cmd.cur_ff, T_MIN, T_MAX)
        # 数据类型转换
        pos_send = float_to_uint(pos_send, P_MIN, P_MAX, 16)
        vel_send = float_to_uint(vel_send, V_MIN, V_MAX, 12)
        KP_send = float_to_uint(KP_send, KP_MIN, KP_MAX, 12)
        KD_send = float_to_uint(KD_send, KD_MIN, KD_MAX, 12)
        cur_ff_send = float_to_uint(cur_ff_send, T_MIN, T_MAX, 12)
        # 数据打包
        msg = [];
        msg.append(pos_send>>8)
        msg.append(pos_send&0xFF)
        msg.append(vel_send>>4)
        msg.append(((vel_send&0xF)<<4) | (KP_send>>8))
        msg.append(KP_send&0xFF)
        msg.append(KD_send>>4)
        msg.append(((KD_send&0xF)<<4) | (cur_ff_send>>8))
        msg.append(cur_ff_send&0xFF)
        self.usb2can.send_std(motor.id, msg)
        
    def recvMsg(self):
        """
        遍历canMsgList, 更新电机状态
        """
        list = self.usb2can.canMsgList
        for msg in list:
            id = msg.data[0]
            pos_recv = (msg.data[1]<<8) | msg.data[2]
            vel_recv = (msg.data[3]<<4) | (msg.data[4]>>4)
            cur_recv = ((msg.data[4]&0xF)<<8) | msg.data[5]
            # convert to float
            pos_recv = uint_to_float(pos_recv, P_MIN, P_MAX, 16)
            vel_recv = uint_to_float(vel_recv, V_MIN, V_MAX, 12)
            cur_recv = uint_to_float(cur_recv, T_MIN, T_MAX, 12)
            print("can_id: ", hex(id), "pos: ", pos_recv, "vel: ", vel_recv, "cur: ", cur_recv)
            if id in motor_dict:
                left_right, front_back = motor_dict[id]
                self.motor[left_right][front_back].state.pos = pos_recv
                self.motor[left_right][front_back].state.vel = vel_recv
                self.motor[left_right][front_back].state.cur = cur_recv
            else:
                print("Error: unknown motor id: ", id)
        # clear canMsgList
        self.usb2can.canMsgList.clear()
            
    def zero_pos(self, motor):
        """
        电机位置回0
        """
        self.set_mode(motor, CMD_MOTOR_MODE)        # 需要先将控制参数设为0
        time.sleep(0.1)
        zero_cmd = MotorCtrlCmd()
        self.send_para(motor, zero_cmd)
        time.sleep(0.1)
        
    def start_motor(self, motor):
        """
        启动电机控制
        """
        self.set_mode(motor, CMD_MOTOR_MODE)

    def stop_motor(self, motor):
        """
        停止电机控制
        """
        self.set_mode(motor, CMD_RESET_MODE)


# 功能测试
def signal_handler(signal, frame):
    sys.exit(0)
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler) # ctrl+c 退出
    print("List of enabled UART:")
    os.system('ls /dev/tty[a-zA-Z]*')
    baudrate = BOUND
    uart_dev= UART_DEV
    ctrl = HT_04_Ctrl(uart_dev, baudrate)
    test_ctrl_cmd = MotorCtrlCmd()
    test_ctrl_cmd.KP = 5
    test_ctrl_cmd.KD = 0.2
    test_ctrl_cmd.cur_ff = 0
    flag = 1;
    while (True):
        # position control test
        test_ctrl_cmd.pos_tar += 0.1*flag
        if test_ctrl_cmd.pos_tar > P_MAX:
            flag = -1
        if test_ctrl_cmd.pos_tar < P_MIN:
            flag = 1
        
        # # velocity control test
        # test_ctrl_cmd.vel_tar = 1
        
        ctrl.send_para(ctrl.motor[LEFT][FRONT], test_ctrl_cmd)
        ctrl.send_para(ctrl.motor[LEFT][BACK], test_ctrl_cmd)
        ctrl.send_para(ctrl.motor[RIGHT][FRONT], test_ctrl_cmd)
        ctrl.send_para(ctrl.motor[RIGHT][BACK], test_ctrl_cmd)
        ctrl.recvMsg()
        time.sleep(1)