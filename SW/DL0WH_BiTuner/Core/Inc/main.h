/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <sys/_stdint.h>

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ADC2_IN1_FWDREV_Pin GPIO_PIN_0
#define ADC2_IN1_FWDREV_GPIO_Port GPIOC
#define ADC3_IN3_VDIODE_Pin GPIO_PIN_2
#define ADC3_IN3_VDIODE_GPIO_Port GPIOC
#define GPIO_SWR_SEL_FWD_Pin GPIO_PIN_3
#define GPIO_SWR_SEL_FWD_GPIO_Port GPIOC
#define GPIO_SWR_SEL_REV_Pin GPIO_PIN_3
#define GPIO_SWR_SEL_REV_GPIO_Port GPIOA
#define GPIO_SPI_RST_Pin GPIO_PIN_4
#define GPIO_SPI_RST_GPIO_Port GPIOC
#define GPIO_SPI_SEL_L_Pin GPIO_PIN_0
#define GPIO_SPI_SEL_L_GPIO_Port GPIOB
#define GPIO_SPI_SEL_C_Pin GPIO_PIN_1
#define GPIO_SPI_SEL_C_GPIO_Port GPIOB
#define GPIO_SPI_SEL_EXT_Pin GPIO_PIN_2
#define GPIO_SPI_SEL_EXT_GPIO_Port GPIOB
#define GPIO_SPI_PWM_Pin GPIO_PIN_11
#define GPIO_SPI_PWM_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

/* USER CODE BEGIN Private defines */

#ifndef PI
# define PI                                                   3.14159265358979f
#endif

#ifndef min
# define min(a,b)                                             (a) < (b) ?  (a) : (b)
#endif

#ifndef max
# define max(a,b)                                             (a) > (b) ?  (a) : (b)
#endif

#ifndef false
# define false 0
#endif
#ifndef true
# define true 1
#endif


#define BITUNER_CTRL_VERSION                                 20190127UL


typedef enum POWERSWITCH_ENUM {

  POWERSWITCH__BAT_SW                                         = 1,
  POWERSWITCH__BAT_HICUR,

  POWERSWITCH__SLOWER_24MHZ,

} POWERSWITCH_ENUM_t;

typedef enum SYSCLK_CONFIG_ENUM {

  SYSCLK_CONFIG_04MHz_MSI                                     =  4000,
  SYSCLK_CONFIG_08MHz_MSI                                     =  8000,
  SYSCLK_CONFIG_16MHz_MSI                                     = 16000,
  SYSCLK_CONFIG_24MHz_MSI                                     = 24000,
  SYSCLK_CONFIG_48MHz_MSI                                     = 48000,
  SYSCLK_CONFIG_80MHz_MSI16_PLL                               = 80000,

} SYSCLK_CONFIG_t;

uint32_t crcCalc(const uint32_t* ptr, uint32_t len);


uint8_t sel_u8_from_u32(uint32_t in_u32, uint8_t sel);
void mainCalcFloat2IntFrac(float val, uint8_t fracCnt, int32_t* outInt, uint32_t* outFrac);
float mainCalc_fwdRev_mV(float adc_mv, float vdiode_mv);
float mainCalc_VSWR(float fwd, float rev);
float mainCalc_mV_to_mW(float mV);
void mainPowerSwitchDo(POWERSWITCH_ENUM_t sw, uint8_t enable);
void mainPowerSwitchInit(void);
void SystemResetbyARMcore(void);

void Again_SystemClock_Config(SYSCLK_CONFIG_t sel);

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
