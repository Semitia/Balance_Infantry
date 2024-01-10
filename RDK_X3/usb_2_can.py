"""
This a library to use USB2CAN module.
author: Leonaruic
GitHub: github.com/semitia
date: 2023-12-28
version: 0.0.1
"""

import sys
import signal
import os
import time
import signal
import threading
import serial
import serial.tools.list_ports
import numpy as np
import binascii # 用于二进制和十六进制的转换

def add_tail(cmd):
    # 在指令后面添加0x0D 0x0A作为帧尾
    cmd.extend([0x0D, 0x0A])
    return cmd


class CanMsg:
    def __init__(self, can_id, data, data_len):
        self.can_id = can_id
        self.data = data
        self.data_len = data_len


class USB2CAN:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
        except Exception as e:
            print("USB2CAN init failed, error: ", e)
            return
        print("USB2CAN init successfully with port:", port, "baud:", baud)
        
        self.rxbuf = bytearray()
        self.end_flag = 0
        self.ReadPortThread = threading.Thread(target=self.read_port)
        self.ReadPortThread.start()
        self.ack_ok = False
        self.canMsgList = []
        
    def read_port(self):
        while True:
            # print("in_waiting: ", self.ser.in_waiting)
            if self.ser.in_waiting > 0:
                msg = self.ser.read(self.ser.in_waiting)
                # print(msg)
                for byte in msg:
                    self.process_byte(byte)
            time.sleep(0.001)
        
    def process_byte(self, byte):
        """处理单个字节
        Args:
            byte (byte): 一个字节
        """
        self.rxbuf.append(byte)
        # 结束标识符 0x0d 0x0a
        if self.end_flag == 0:
            if byte == 0x0d:
                self.end_flag = 1
        elif self.end_flag == 1:
            if byte == 0x0a:
                self.end_flag = 0
                # print("received ", binascii.hexlify(self.rxbuf))
                self.process_frame()
            else:
                self.end_flag = 0
                self.rxbuf.clear()
                print("Error: end_flag = 1, byte != 0x0a")
        return
    
    def process_frame(self):
        """处理一个数据包
        """
        if self.rxbuf[0] == 0x4f and self.rxbuf[1] == 0x4b: # OK
            self.ack_ok = True
        elif self.rxbuf[0] == 0x41 and self.rxbuf[1] == 0x54: # AT: can数据
            can_id = (self.rxbuf[2] << 3) | (self.rxbuf[3] >> 5)
            data_len = self.rxbuf[6]
            data = self.rxbuf[7:7+data_len]
            new_can_msg = CanMsg(can_id, data, data_len)
            self.canMsgList.append(new_can_msg)
            # print("can_id: ", hex(can_id), "data_len: ", data_len, "data: ", binascii.hexlify(data))
        self.rxbuf.clear()    
    
    def wait_ok_ack(self, action):
        """等待回复OK
        Args:
            action (string): 需要等待回复的行为
        """
        count = 50
        while not self.ack_ok:
            count -= 1
            time.sleep(0.001)
            if count <= 0:
                print(action, " timeout!")
                return
        self.ack_ok = False
        print(action, " ok!")
    
    def set_ATmode(self):
        # 设置AT模式
        cmd = "AT+AT\r\n"
        print("set_ATmode: ", binascii.hexlify(cmd.encode('UTF-8')))
        self.ser.write(cmd.encode('UTF-8'))
        self.wait_ok_ack("set_ATmode")
    
    def send_std(self, can_id, data):
        # 发送CAN标准帧
        cmd = bytearray([0x41, 0x54])       # 添加帧头‘AT'
        # 报文标识符
        cmd.append((can_id >> 3) & 0xFF)    # 取id高八位
        cmd.append((can_id << 5) & 0xFF)    # 取id低三位
        cmd.append(0x00)                    # 拓展帧id位均为0
        cmd.append(0x00)

        cmd.append(len(data))               # 数据长度
        cmd.extend(data)                    # 数据
        # 添加帧尾
        cmd = add_tail(cmd)
        self.ser.write(cmd)
        # print("send_std: ", binascii.hexlify(cmd))


# 功能测试
def signal_handler(signal, frame):
    sys.exit(0)
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler) # ctrl+c 退出
    print("List of enabled UART:")
    os.system('ls /dev/tty[a-zA-Z]*')
    uart_dev= "/dev/ttyUSB0" # input("请输出需要测试的串口设备名:")
    baudrate = 2000000 # input("请输入波特率(9600,19200,38400,57600,115200,921600):")
    usb2can = USB2CAN(uart_dev, int(baudrate))
    usb2can.set_ATmode()
    while True:
        usb2can.send_std(0x63, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        time.sleep(1)
    