/***************************************************************** 
*  @brief:         
*  @File:          
*  @Project:     
*  @Author:      
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
#include "Interrupt.h"

void Interrupt_Init(void){
  
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
  if(htim == (&htim1)){
    Tim_Control[TIM1_ID].ISR_Func();
  }else if(htim ==(&htim6)){
    Tim_Control[TIM6_ID].ISR_Func();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart){
  if(huart == (&huart1)){
    __HAL_DMA_DISABLE(huart->hdmatx);  
    Uart_Control[USART1_ID].Transmit_Over_Flag = DMA_SendOver;
    Uart_Control[USART1_ID].RX_Tx_Flag     =0;
  }else if(huart ==(&huart3)){
    __HAL_DMA_DISABLE(huart->hdmatx);  
    Uart_Control[USART3_ID].Transmit_Over_Flag = DMA_SendOver;    
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
  if(huart == (&huart1)){
    ISR_Usart1_Rx_Task();
    HAL_UART_Receive_IT(&huart1,Uart_Control[USART1_ID].RX_Buffer,1);
  }else if(huart ==(&huart3)){ 
    ISR_Usart3_Rx_Task();
  }
}
