/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
//#define USART1_BaudRate 921600
//#define USART3_BaudRate 9600

#define BARO_PS_Pin GPIO_PIN_3
#define BARO_PS_GPIO_Port GPIOE
#define BARO_SCK_Pin GPIO_PIN_2
#define BARO_SCK_GPIO_Port GPIOE
#define IMU_SDO0_Pin GPIO_PIN_8
#define IMU_SDO0_GPIO_Port GPIOB
#define AT_CON_Pin GPIO_PIN_5
#define AT_CON_GPIO_Port GPIOB
#define BARO_CSB_Pin GPIO_PIN_4
#define BARO_CSB_GPIO_Port GPIOE
#define BARO_SDO_Pin GPIO_PIN_5
#define BARO_SDO_GPIO_Port GPIOE
#define BARO_SDI_Pin GPIO_PIN_6
#define BARO_SDI_GPIO_Port GPIOE
#define IMU_SDO1_Pin GPIO_PIN_9
#define IMU_SDO1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_6
#define IMU_CS_GPIO_Port GPIOA
#define IMU_SDI_Pin GPIO_PIN_5
#define IMU_SDI_GPIO_Port GPIOA
#define IMU_SDO4_Pin GPIO_PIN_12
#define IMU_SDO4_GPIO_Port GPIOB
#define IMU_SDO5_Pin GPIO_PIN_13
#define IMU_SDO5_GPIO_Port GPIOB
#define IMU_SCLK_Pin GPIO_PIN_7
#define IMU_SCLK_GPIO_Port GPIOA
#define IMU_SDO2_Pin GPIO_PIN_10
#define IMU_SDO2_GPIO_Port GPIOB
#define IMU_SDO3_Pin GPIO_PIN_11
#define IMU_SDO3_GPIO_Port GPIOB
#define IMU_SDO6_Pin GPIO_PIN_14
#define IMU_SDO6_GPIO_Port GPIOB
#define IMU_SDO7_Pin GPIO_PIN_15
#define IMU_SDO7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
