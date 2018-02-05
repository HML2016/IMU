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
#if defined(UART_SIGLE_BYTE_MODE)

static uint16_t uart1_check_valid_count=0;
static uint16_t uart1_rx_count=0;
static uint8_t Buffer_Command[COMMAND_NUM]={0};
static uint8_t  header[HEADER_LENGTH]={0};
#endif
static uint8_t  uart1_check=0;
static uint16_t Current_Command_Id=0;
static uint8_t Command_Flag=Command_InValid;

void ISR_Usart1_Rx_Task(void){
#if defined(UART_SIGLE_BYTE_MODE)

#elif defined(UART_MULTI_BYTE_MODE)
 
  

#endif
}

void ISR_Usart1_Tx_Task(void){
  //HAL_UART_Transmit_DMA(&huart1, Uart_Control[USART1_ID].TX_Buffer, Uart_Control[USART1_ID].TX_Size);
}

void ISR_Usart3_Rx_Task(void){
  Task_Seque_Par.Activate(TASK_TRANSIT_DEV);
}

void ISR_Usart3_Tx_Task(void){
  //HAL_UART_Transmit_DMA(&huart3, Uart_Control[USART3_ID].TX_Buffer, Uart_Control[USART3_ID].TX_Size);
}
/***************************************************************** 
*  @brief:          
*  @Function: 
*  @inparam:
*  @outparam:  
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description: the based clock source is 100khz                 
*****************************************************************/
void ISR_Tim1_Task(void){
  System_Work_Info.Time_Int++;
  if((System_Work_Info.Time_Int%500)==0){    //ad sample is 2kHz
    Task_Seque_Par.Activate(TASK_SENSOR_READ);
    Task_Seque_Par.Activate(TASK_DEV_CALI);
    Task_Seque_Par.Activate(TASK_INS_LOAD_DATA);
    Task_Seque_Par.Activate(TASK_INS_BUFFER_DATA);
    if((INS_Nav_Par.AlignData[5]==1)||(System_Work_Info.System_Time>3000)){
      Task_Seque_Par.Activate(TASK_INS_CAL);
      Task_Seque_Par.Activate(TASK_PDR_CAL);
    }else if(INS_Nav_Par.AlignData[5]==0){
      Task_Seque_Par.Activate(TASK_ALIGN);
      }
//    Task_Seque_Par.Activate(TASK_PDR_CAL);
  }
}
/***************************************************************** 
*  @brief:          
*  @Function: 
*  @inparam:
*  @outparam:  
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description: the based clock source is 1khz, put the data to the pc
*****************************************************************/
void ISR_Tim6_Task(void){
  /*1ms*/
  System_Work_Info.Time_Inc++;
  if(System_Work_Info.Time_Inc>=Dev_Set_Info.Output_SPS){
    System_Work_Info.Time_Inc=0;
    Task_Seque_Par.Activate(TASK_TRANSMIT_DATA); //load the transmit data to pc
  }
}
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
void IDLE_Usart1_Rx_Task(void)  
{  
    uint32_t temp=0;  

    if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != SET)){   
      __HAL_UART_CLEAR_IDLEFLAG(&huart1);  
      HAL_UART_DMAStop(&huart1);  
      temp = huart1.hdmarx->Instance->NDTR;  
      Uart_Control[USART1_ID].Current_Rx_len =  Uart_Control[USART1_ID].RX_Size - temp - 1 ;   
      Uart_Control[USART1_ID].Recive_Over_Flag=1;
      ISR_Usart1_Rx_Task();
      HAL_UART_Receive_DMA(&huart1, Uart_Control[USART1_ID].RX_Buffer, Uart_Control[USART1_ID].RX_Size); 
    }          
} 
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
void IDLE_Usart3_Rx_Task(void)  
{  
    uint32_t temp;  
  
    if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != SET))  
    {   
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);  
        HAL_UART_DMAStop(&huart3);  
        temp = huart3.hdmarx->Instance->NDTR;  
        Uart_Control[USART3_ID].Current_Rx_len =  Uart_Control[USART3_ID].RX_Size - temp;   
        Uart_Control[USART3_ID].Recive_Over_Flag=1;
        ISR_Usart3_Rx_Task();
        HAL_UART_Receive_DMA(&huart3, Uart_Control[USART3_ID].RX_Buffer, Uart_Control[USART3_ID].RX_Size); 
    }  
} 

