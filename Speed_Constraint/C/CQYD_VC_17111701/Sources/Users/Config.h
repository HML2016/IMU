/***************************************************************** 
*  @brief:       All the software and hardware initial  
*  @File:        Config.h  
*  @Project:     
*  @Author:      Liu Ning
*  @Date:        170613
*  @CopyRight:   
*  @Version:     V1.0
*  @Description: 
*****************************************************************/
#ifndef __CONFIG_H
#define __CONFIG_H
/*-select the device platform-*/
#if defined(STM32F429xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F767xx)
  #include "stm32f7xx_hal.h"
#endif

/*-include the driver file-*/
#include "GlobalDefine.h"
#include "Interrupt.h"

#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "SensorRead.h"
#include "ExtDevCon.h"

#define UART_BUFFER_MAX_LENGTH 512    //define the max value of uart buffer
#define UART_NUM 2
#define USART1_ID 0
#define USART3_ID 1

typedef struct{
  UART_InitTypeDef Init;
  uint8_t RX_Buffer[UART_BUFFER_MAX_LENGTH];
  uint8_t TX_Buffer[UART_BUFFER_MAX_LENGTH];
  uint16_t RX_Size;
  uint16_t TX_Size;
  void (*Tx_ISR_Func) (void);        //uart transmit isr
  void (*Rx_ISR_Func) (void);
  
  void (*IDLE_Rx_ISR_Func) (void);   //leisure receive and solve
  uint16_t Current_Rx_len;   //current receive length in the idle function
  
  uint8_t  Recive_Over_Flag;
  uint8_t  Transmit_Over_Flag;
  
  uint8_t RX_Tx_Flag;
  
  uint8_t *SendP;
}Uart_Info;

extern Uart_Info Uart_Control[UART_NUM];
/*set the usart baud*/
extern uint32_t USART1_BaudRate;
extern uint32_t USART3_BaudRate;

enum USART_DMA_STATE{
  DMA_Sending=0,
  DMA_SendOver
};  

#define TIM_NUM 2
#define TIM1_ID 0
#define TIM6_ID 1

typedef struct{
  TIM_HandleTypeDef Init;
  void (*ISR_Func) (void);
}Timer_Info;

extern Timer_Info Tim_Control[TIM_NUM];
void System_Init(void);     //System Init
void Config_Init(void);
#endif
