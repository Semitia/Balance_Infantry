// #ifndef _IMU_UART_H
// #define _IMU_UART_H

// #include <stm32f4xx.h>
// #include <stm32f4xx_conf.h>
// #include <string.h>
// #include <stdint.h>
// #include <math.h>
// #include <stdio.h>
// #include <stdlib.h>

// #include "algorithmOfCRC.h"

// #define IMU_UART_GPIOx_CLOCK_CMD RCC_AHB1PeriphClockCmd
// #define IMU_UART_RCC_AxBxPeriph_GPIOx RCC_AHB1Periph_GPIOC
// #define IMU_UART_UARTx_CLOCK_CMD RCC_APB1PeriphClockCmd
// #define IMU_UART_RCC_AxBxPeriph_UARTx RCC_APB1Periph_UART4
// #define IMU_UART_GPIOx GPIOC
// #define IMU_UART_UARTx UART4
// #define IMU_UART_GPIO_AF_USARTx GPIO_AF_UART4
// #define IMU_UART_GPIO_PinSource GPIO_PinSource11
// #define IMU_UART_GPIO_Pin_x GPIO_Pin_11
// #define IMU_UART_BaudRate 921600
// #define IMU_UART_IRQn UART4_IRQn
// #define IMU_UART_IRQHandler UART4_IRQHandler

// #define IMU_UART_RECV_DMAx_Streamx DMA1_Stream2
// #define IMU_UART_RECV_RCC_AxBxPeriphClockCmd RCC_AHB1PeriphClockCmd
// #define IMU_UART_RECV_RCC_AxBxPeriph_DMAx RCC_AHB1Periph_DMA1
// #define IMU_UART_RECV_DMA_Channel_x DMA_Channel_4
// #define IMU_UART_RECV_DMA_FLAG_TCIFx DMA_FLAG_TCIF2
// #define IMU_UART_RECV_DMA_IT_TCIFx DMA_IT_TCIF2

// /* 其它GPIO配置到特定位置修改 */
// /* 其它UART配置到特定位置修改 */
// /* 其它NVIC配置到特定位置修改 */
// /* 其它DMA配置到特定位置修改 */

// void IMU_UART_Configuration(void);

// #pragma pack(push, 1)
// typedef struct IMUDataReceive
// {
//     uint8_t start_flag;
//     short gyro[3];
//     short accel[3];
//     uint16_t crc16;
// } IMUDataReceive;
// #pragma pack(pop)

// #define IMU_RECEIVE_NUM sizeof(IMUDataReceive)

// #endif // !_IMU_UART_H
