#include <stm32f4xx.h>
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

// IO口地址映射 适合F405
#define GPIOA_ODR_Addr (GPIOA_BASE + 20) // 0x40010814
#define GPIOB_ODR_Addr (GPIOB_BASE + 20) // 0x40010C14
#define GPIOC_ODR_Addr (GPIOC_BASE + 20) // 0x40011014
#define GPIOD_ODR_Addr (GPIOD_BASE + 20) // 0x40011414
#define GPIOE_ODR_Addr (GPIOE_BASE + 20) // 0x40011814
#define GPIOF_ODR_Addr (GPIOF_BASE + 20) // 0x40011A14
#define GPIOG_ODR_Addr (GPIOG_BASE + 20) // 0x40011E14

#define GPIOA_IDR_Addr (GPIOA_BASE + 16) // 0x40010810
#define GPIOB_IDR_Addr (GPIOB_BASE + 16) // 0x40010C10

#define GPIOC_IDR_Addr (GPIOC_BASE + 16) // 0x40011010
#define GPIOD_IDR_Addr (GPIOD_BASE + 16) // 0x40011410
#define GPIOE_IDR_Addr (GPIOE_BASE + 16) // 0x40011810
#define GPIOF_IDR_Addr (GPIOF_BASE + 16) // 0x40011A10
#define GPIOG_IDR_Addr (GPIOG_BASE + 16) // 0x40011E10

// 适合F405
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) //输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  //输入

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) //输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  //输入

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) //输出
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  //输入

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) //输出
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  //输入

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) //输出
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  //输入

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) //输出
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  //输入

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) //输出
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  //输入
