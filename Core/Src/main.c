/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"

/*
程序结构:
1. 程序的起始必须以扇区对齐
2. 程序结束位置的前一个字,存储当前程序的crc校验数据
*/

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

// bootinfo结构体大小
#define BootInfo_STRUCT_SIZE sizeof(BootInfo_t)

// 根据程序引导信息,找到存储crc校验值的地址
#define FIND_CRC_ADR(start, size) (((uint32_t)start + (uint32_t)size - 4UL) & 0xFFFFFFFBUL)

// FLASH定义
#define FLASH_ADDRESS_BASE ((uint32_t)0x08000000)
#define FLASH_SIZE ((uint32_t)0x00010000)
#define FLASH_END ((uint32_t)(FLASH_ADDRESS_BASE + FLASH_SIZE))
// SRAM定义
#define RAM_BASE ((uint32_t)0x20000000)
#define RAM_SIZE ((uint32_t)0x00005000)
#define RAM_END ((uint32_t)(RAM_BASE + RAM_SIZE))
// 引导信息存储区定义
#define PAGE_SIZE ((uint32_t)0x00000400)														 //	1K
#define ENTRY_OFFSET ((uint32_t)0x00000001)													 // 程序入口相对程序起始地址的偏移量
#define FLASH_START_BASE ((uint32_t)0x08000000)											 // flash 起始地址
#define BOOT_INFO_START ((uint32_t)0x08000400)											 // info 起始地址
#define BOOT_INFO_SIZE ((uint32_t)0x00000400)												 // info 大小
#define BOOT_INFO_END ((uint32_t)(BOOT_INFO_START + BOOT_INFO_SIZE)) // info 结束地址

// jump to target address
#if defined(__GNUC__)
void prvJump(uint32_t sp, uint32_t pc)
{
	__asm__("MOV SP, R0");
	__asm__("BX R1");
}
#else
__asm static void prvJump(uint32_t sp, uint32_t pc)
{
	MOV SP, R0
	BX R1
}
#endif

// send data use usart
// xUsart: usart number
// puc: data point
// ulSize: data size of byte
static void prvUsartSendData(USART_TypeDef *xUsart, uint8_t *puc, uint32_t ulSize)
{
	/* loop send data */
	while (ulSize--)
	{
		/* wait transfer complete */
		while (!LL_USART_IsActiveFlag_TC(xUsart))
			;
		/* write data to usart data register */
		LL_USART_TransmitData8(xUsart, *(puc++));
	}
}

// receive data byte usart
// xUsart: usart number
// puc: data point
// ulSize: data size of byte
static void prvUsartReceiveData(USART_TypeDef *xUsart, uint8_t *puc, uint32_t ulSize)
{
	/* check usart1 receive data */
	while (!LL_USART_IsActiveFlag_RXNE(xUsart))
		;
	/* read data in usart data register */
	*(puc++) = LL_USART_ReceiveData8(xUsart);
}

// crc check
// bit0: 0: boot is correct; 1: boot is error
// bit1: 0: app is correct; 1: app is error
static uint8_t prvCheckCrc(const void *p)
{
	uint8_t ucResult = 0;
	xProgramRealInfo_t *xpInfo = (xProgramRealInfo_t *)p;
	uint32_t *ulpAddress[2] = {0};
	uint32_t ulSize[2] = {0};
	uint32_t data = 0;
	uint32_t index = 0;

	/* compute start address */
	ulpAddress[0] = (uint32_t *)xpInfo->ulBootBase;
	ulpAddress[1] = (uint32_t *)xpInfo->ulAppBase;
	/* compute program size */
	ulSize[0] = (uint32_t)xpInfo->ulBootSize;
	ulSize[1] = (uint32_t)xpInfo->ulAppSize;

	/* compute crc */
	for (int i = 0; i < 2; i++)
	{
		/* reset crc unit */
		LL_CRC_ResetCRCCalculationUnit(CRC);
		/* check bootloader and application crc */
		data = 0;
		index = 0;
		/* Compute the CRC of Data Buffer array*/
		for (index = 0; index < (((ulSize[i]) / 4) - 1); index++) // the last word is crc value
		{
			data = (uint32_t)(*(ulpAddress[i]));
			LL_CRC_FeedData32(CRC, data);
			(ulpAddress[i])++;
		}
		/* find program crc value address(when writed) */
		data = FIND_CRC_ADR(ulpAddress[i], ulSize[i]);

		/* Return computed CRC value and compare */
		if (LL_CRC_ReadData32(CRC) != data)
		{
			ucResult |= (1 << i);
		}
	}

	/* return result */
	return ucResult;
}

// config usart
static void prvConfigUsart1(void)
{
	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	/*
	USART1 GPIO Configuration
	PA9   ------> USART1_TX
	PA10   ------> USART1_RX
	*/
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_FLOATING);

	//	LL_USART_InitTypeDef USART_InitStruct = { 0 };
	//	USART_InitStruct.BaudRate = 115200;
	//	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	//	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	//	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	//	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	//	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	//	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

	/* usart1 hardware config */
	USART1->CR1 = 0x0000200CUL;
	USART1->CR2 = 0UL;
	USART1->CR3 = 0UL;
	/* config usart1 baudrate */
	USART1->BRR = 0x00000045;
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* close all Interruption */
	__ASM("CPSID	I");
	
#if 0
	/* clear all sram */
	uint32_t * ulp = (uint32_t *)RAM_BASE;
	
	while((uint32_t)ulp < RAM_END)	
		*(ulp++) = 0UL;
#endif

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* Peripheral clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

	/* select boot info to new */
	xBootInfo_t *xpBoot = (xBootInfo_t *)BOOT_INFO_START;

	while (eIsNo == xpBoot->usIsNext && (uint32_t)xpBoot < BOOT_INFO_END)
	{
		// do something
		xpBoot++;
	}

	/* Verify the boot address */
	if ((uint32_t)xpBoot < BOOT_INFO_END)
	{
		xProgramRealInfo_t xRealInfo = {0};
		/* boot start address */
		xRealInfo.ulBootBase = (uint32_t *)(((uint32_t)(xpBoot->xBootlader.usStartAddress) * PAGE_SIZE) + FLASH_START_BASE);
		/* boot size */
		xRealInfo.ulBootSize = (uint32_t)((uint32_t)(xpBoot->xBootlader.usSize) * PAGE_SIZE);
		/* app start address */
		xRealInfo.ulAppBase = (uint32_t *)(((uint32_t)(xpBoot->xApplication.usStartAddress) * PAGE_SIZE) + FLASH_START_BASE);
		/* app size */
		xRealInfo.ulAppSize = (uint32_t)((uint32_t)(xpBoot->xApplication.usSize) * PAGE_SIZE);

		/* get reset flag */
		uint32_t ulResetFlag = RCC->CSR;
		/* clear reset flag */
		LL_RCC_ClearResetFlags();

		/* check address */
		if ((uint32_t)xRealInfo.ulBootBase >= FLASH_END || (uint32_t)xRealInfo.ulBootBase < FLASH_ADDRESS_BASE 
				|| (uint32_t)xRealInfo.ulAppBase >= FLASH_END || (uint32_t)xRealInfo.ulAppBase < FLASH_ADDRESS_BASE)
		{
			/* illegal address */
			goto StartUpFail;
		}

		/* jump to boot/app */
		if (!prvCheckCrc((void *)&xRealInfo))
		{
			/* Jump according to the reset source */
			if ((RCC_CSR_SFTRSTF) == (ulResetFlag & (RCC_CSR_SFTRSTF)))
			{
				/* software reset, maybe need update */
				LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_BKP);

				/*
						read bkp data, and judge target.

						LL_RTC_BKP_DR1: Used to jump between states
							- boot = 1, app = 2;
							- 0x0102: from boot to app
							- 0x0201: from app to boot
							- other value is illegal
				*/
				switch (LL_RTC_BKP_GetRegister(BKP, LL_RTC_BKP_DR1))
				{
				case 0x0102:
				{
					/* jump to app */
					prvJump(*(xRealInfo.ulAppBase), *(xRealInfo.ulAppBase + 4));
				}
				break;
				case 0x0201:
				{
					/* jump to boot */
					prvJump(*(xRealInfo.ulBootBase), *(xRealInfo.ulBootBase + 4));
				}
				break;
				default:
					/* illegal */
					break;
				}
			}
			/* pin reset, POR/PDR reset, Low-power reset is normal start */
			if (((RCC_CSR_PINRSTF) | (RCC_CSR_LPWRRSTF) | (RCC_CSR_PORRSTF)) ==
					(ulResetFlag & ((RCC_CSR_PINRSTF) | (RCC_CSR_LPWRRSTF) | (RCC_CSR_PORRSTF))))
			{
				/* jump to application */
				prvJump(*(xRealInfo.ulAppBase), *(xRealInfo.ulAppBase + 4));
			}
			/* other reset is wrong!!! */
		}
	}

StartUpFail:

	/* config usart */
	prvConfigUsart1();

	/* boot/app are error */
	while (1)
	{
		uint8_t val = 0;
		prvUsartReceiveData(USART1, &val, 1UL);
		prvUsartSendData(USART1, &val, 1UL);
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
