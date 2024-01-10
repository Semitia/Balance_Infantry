# 平衡步兵

## 电路测试接口

电路测试主要是CAN1与CAN2，留了TEST_TASK_ON宏定义来给电路测试板子的CAN口有没有问题。

## 轮毂电机参数辨识接口

MF9025_IDENTIFY_ON为1时表示打开轮毂电机参数辨识接口，注意此时需要将机器人用凳子架空，轮子不着地，详细原理看《轮毂电机参数辨识.pdf》。

## RTT Viewer 接口