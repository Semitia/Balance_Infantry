/**
 ******************************************************************************
 * @file    tools.c
 * @brief   通用小工具
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "tools.h"

/**
 * @brief 毫秒级延时
 * @param[in] t /ms
 */
void delay_ms(unsigned long t)
{
    int i;
    for (i = 0; i < t; i++)
    {
        int a = 10300;
        while (a--)
            ;
    }
}
/**
 * @brief 微秒级延时
 * @param[in] t / us
 */
void delay_us(unsigned long t)
{
    int i;
    for (i = 0; i < t; i++)
    {
        int a = 37;
        while (a--)
            ;
    }
}
