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
// ������Ϣ�ṹ��
typedef struct xProgramInfo_t
{
	uint16_t usStartAddress; // ������ʼ��ַ,��������
	uint16_t usSize;				 // �����С,��������
} xProgramInfo_t;

// ������Ϣ�ṹ����Ч��־ö��
typedef enum Effecient_t
{
	eIsNo = 0xA55A,	 // ��Ч
	eIsYes = 0xFFFF, // ��Ч

	SizeLimit = 0xFFFF
} Effecient_t;

// ������Ϣ�ṹ��
typedef struct xBootInfo_t
{
	Effecient_t usIsNext;				 // ��Ч�Ա�־
															 // 0xFFFF: ��ǰ�ṹ����Ч; 0xA55A: ��ǰ�ṹ����Ч,��Ҫ��ѯ��һ��λ��
	uint16_t usExtend;					 // ��չ��־
	xProgramInfo_t xBootlader;	 // bootloader������Ϣ
	xProgramInfo_t xApplication; // boot������Ϣ
	uint32_t usExtend2;					 // �������չ��־
} xBootInfo_t;

// ����ĵ�ַ,��С��Ϣ�ṹ��
typedef struct xProgramRealInfo_t
{
	uint32_t *ulBootBase; // boot ��ʼ��ַ
	uint32_t ulBootSize;	// boot ��С
	uint32_t *ulAppBase;	// app ��ʼ��ַ
	uint32_t ulAppSize;		// app ��С
} xProgramRealInfo_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
// bootinfo�ṹ���С
#define BootInfo_STRUCT_SIZE 			sizeof(BootInfo_t)

// ���ݳ���������Ϣ,�ҵ��洢crcУ��ֵ�ĵ�ַ
#define FIND_CRC_ADR(start, size) (((uint32_t)start + (uint32_t)size - 4UL) & 0xFFFFFFFBUL)

// FLASH����
#define FLASH_ADDRESS_BASE 				((uint32_t)0x08000000)
#define FLASH_SIZE 								((uint32_t)0x00010000)
#define FLASH_END 								((uint32_t)(FLASH_ADDRESS_BASE + FLASH_SIZE))
// SRAM����
#define RAM_BASE 									((uint32_t)0x20000000)
#define RAM_SIZE 									((uint32_t)0x00005000)
#define RAM_END 									((uint32_t)(RAM_BASE + RAM_SIZE))
// ������Ϣ�洢������
#define PAGE_SIZE 								((uint32_t)0x00000400)													// 1K
#define ENTRY_OFFSET 							((uint32_t)0x00000001)													// ���������Գ�����ʼ��ַ��ƫ����
#define FLASH_START_BASE 					((uint32_t)0x08000000)											 		// flash ��ʼ��ַ
#define BOOT_INFO_START 					((uint32_t)0x08000400)											 		// info ��ʼ��ַ
#define BOOT_INFO_SIZE 						((uint32_t)0x00000400)													// info ��С
#define BOOT_INFO_END 						((uint32_t)(BOOT_INFO_START + BOOT_INFO_SIZE))	// info ������ַ

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
