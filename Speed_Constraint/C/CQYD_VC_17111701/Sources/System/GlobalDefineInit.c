/***************************************************************** 
*  @brief:       the system global initial, focus on the system set, system work, dev set.
*  @File:        GlobalDefineInit.c
*  @Project:     HDNT IMU
*  @Author:      liu ning
*  @Date:        20170718
*  @CopyRight:   Copyright (c) 2017 Beijing Information Science & Technology University, 
                                    Beijing Key Laboratory of Hign Dynamic Navigation Technology
                 ISC License (open source)
*  @Version:     17071801
*  @Description: system initial
*****************************************************************/
#include "GlobalDefine.h"

Dev_Work_Info Dev_Set_Info = {RunInIMU, 5, 921600, 1, PlaceSingleDev,0,6,0,0,2,ProtIEEE754,
                              1,16,{0},{0},{0}
                              };

System_State_Info System_Work_Info = {0,0,0,0,0,0,0,0};

/***************************************************************** 
*  @brief:       this function must in the first in the main.c
*  @Function: 
*  @inparam:
*  @outparam:  
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void GlobalDefine_Init(void){
  /*initial the system time config*/

} 
/***************************************************************** 
*  @brief:      System Soft Reset
*  @Function: 
*  @inparam:
*  @outparam:  
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void Sys_Soft_Reset(void)
{  
    SCB->AIRCR =0X05FA0000|(uint32_t)0x04;      
}

