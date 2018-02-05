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
#ifndef __SIMPARSET_H
#define __SIMPARSET_H

#if defined(STM_RUN)
  #if defined(STM32F429xx)
    #include "stm32f4xx_hal.h"
  #elif defined(STM32F767xx)
    #include "stm32f7xx_hal.h"
  #endif
#elif defined(VS_SIM)
  #define HAL
  
#endif



#endif

