/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_ll_crc.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_rtc.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
#include "usart.h"
#include "FlashOS.h"

/* Exported types ------------------------------------------------------------*/
// 程序信息结构体
typedef struct xProgramInfo_t
{
	uint16_t usStartAddress; // 程序起始地址,以扇区计
	uint16_t usSize;				 // 程序大小,以扇区计
} xProgramInfo_t;

// 引导信息结构体有效标志枚举
typedef enum Effecient_t
{
	eIsNo = 0xA55A,	 // 无效
	eIsYes = 0xFFFF, // 有效

	SizeLimit = 0xFFFF
} Effecient_t;

// 引导信息结构体
typedef struct xBootInfo_t
{
	Effecient_t usIsNext;				 // 有效性标志
															 // 0xFFFF: 当前结构体有效; 0xA55A: 当前结构体无效,需要查询下一个位置
	uint16_t usExtend;					 // 扩展标志
	xProgramInfo_t xBootlader;	 // bootloader引导信息
	xProgramInfo_t xApplication; // boot引导信息
	uint32_t usExtend2;					 // 额外的扩展标志
} xBootInfo_t;

// 程序的地址,大小信息结构体
typedef struct xProgramRealInfo_t
{
	uint32_t *ulBootBase; // boot 起始地址
	uint32_t ulBootSize;	// boot 大小
	uint32_t *ulAppBase;	// app 起始地址
	uint32_t ulAppSize;		// app 大小
} xProgramRealInfo_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
// bootinfo结构体大小
#define BootInfo_STRUCT_SIZE 			sizeof(BootInfo_t)

// 根据程序引导信息,找到存储crc校验值的地址
#define FIND_CRC_ADR(start, size) (((uint32_t)start + (uint32_t)size - 4UL) & 0xFFFFFFFBUL)

// FLASH定义
#define FLASH_ADDRESS_BASE 				((uint32_t)0x08000000)
#define FLASH_SIZE 								((uint32_t)0x00010000)
#define FLASH_END 								((uint32_t)(FLASH_ADDRESS_BASE + FLASH_SIZE))
// SRAM定义
#define RAM_BASE 									((uint32_t)0x20000000)
#define RAM_SIZE 									((uint32_t)0x00005000)
#define RAM_END 									((uint32_t)(RAM_BASE + RAM_SIZE))
// 引导信息存储区定义
#define PAGE_SIZE 								((uint32_t)0x00000400)													// 1K
#define ENTRY_OFFSET 							((uint32_t)0x00000001)													// 程序入口相对程序起始地址的偏移量
#define FLASH_START_BASE 					((uint32_t)0x08000000)											 		// flash 起始地址
#define BOOT_INFO_START 					((uint32_t)0x08000400)											 		// info 起始地址
#define BOOT_INFO_SIZE 						((uint32_t)0x00000400)													// info 大小
#define BOOT_INFO_END 						((uint32_t)(BOOT_INFO_START + BOOT_INFO_SIZE))	// info 结束地址

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
