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
#ifndef __EXTDEVCON_H
#define __EXTDEVCON_H

/*-select the device platform-*/
#if defined(STM32F429xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F767xx)
  #include "stm32f7xx_hal.h"
#endif

#include "Modbus.h"
#include "Config.h"
#include "GlobalDefine.h"

typedef struct{
  float32_t *BufferData;
}ExtDev_Con_Info;

extern ExtDev_Con_Info ExtDev_Par;

void ExtDevCon_Init(void);

#endif

