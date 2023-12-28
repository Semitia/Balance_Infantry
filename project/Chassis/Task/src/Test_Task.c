/**
 ******************************************************************************
 * @file    Test_Task.c
 * @brief   给电路用的测试任务
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "Test_Task.h"

int8_t can_send_test[8];
ExcuteTorque excute_torque_test;

#if TEST_TASK_ON
/**
 * @brief 电路测试任务
 * @param[in] void
 */
void Test_task(void *pvParameters)
{
        portTickType xLastWakeTime;

        float t = 0;

        while (1)
        {

                xLastWakeTime = xTaskGetTickCount();

                CLOLOR_LED_ON;

                // WifiSendData data;
                // BlueToothSendData data;
                // data.fdata[0] = arm_sin_f32(t);
                // data.fdata[1] = arm_sin_f32(2 * t);
                // data.fdata[2] = arm_sin_f32(3 * t);
                // data.fdata[3] = arm_sin_f32(4 * t);

                // BLUE_TOOTHSendData(&data);
                // delay_ms(1);
                t += 0.1f;

                Ozone[0] = arm_sin_f32(t) * 100.0f;
                Ozone[1] = arm_sin_f32(2 * t) * 100.0f;
                Ozone[2] = arm_sin_f32(3 * t) * 100.0f;

                // INA_READ();

                // test pc comm
                SendtoPC();

                // test can1
                CanSend(CAN1, can_send_test, 0x110, 8);

                // test can2
                CanSend(CAN2, can_send_test, 0x111, 8);

                // test rs485_1
                MOTOR1_START_WRITE();
                DMA_Cmd(MOTOR1_RS485_SEND_DMAx_Streamx, ENABLE);

                // // test rs485_2
                MOTOR2_START_WRITE();
                DMA_Cmd(MOTOR2_RS485_SEND_DMAx_Streamx, ENABLE);
                // DMA_Cmd(WIFI_SEND_DMAx_Streamx, ENABLE);
                // delay_ms(2);

                // test 4 knee motor
                // execute_control(&excute_torque_test);

                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        }
}
#endif // DEBUG
