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
#include "Config.h"
/***************************************************************** 
*  @brief:          
*  @Function: 
*  @inparam:
*  @outparam:  
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void System_Init(void){
  
  delay_init(216);
  Sensor_Read_Init();
  System_Work_Info.Product_ID=YEAR;
  
//  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);  //mag reset
  
  HAL_UART_Receive_IT(&huart1,Uart_Control[USART1_ID].RX_Buffer,1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); 
  HAL_UART_Receive_DMA(&huart3, Uart_Control[USART3_ID].RX_Buffer, Uart_Control[USART3_ID].RX_Size);  
//  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); 
    
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start_IT(&htim6); 
    
}

