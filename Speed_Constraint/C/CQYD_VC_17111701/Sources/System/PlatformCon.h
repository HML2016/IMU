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
#ifndef	__PLATFORMCON_H__
#define	__PLATFORMCON_H__

/*-select the device platform-*/
#if defined(STM32F429xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F767xx)
  #include "stm32f7xx_hal.h"
#endif

/*--------------Data Type Declaration---------------------*/
#if defined(LIB_GENERATION)   //this typedef is used to lib generation
	typedef unsigned char uint8_t;   
	typedef char int8_t;
	typedef unsigned short int uint16_t;  
	typedef short int int16_t;   
	typedef long int int32_t;   
	typedef unsigned long int uint32_t;  
#elif defined(VS_SIM)
	typedef unsigned char uint8_t;
	typedef char int8_t;
	typedef unsigned int uint16_t;
	typedef int int16_t;
	typedef long int int32_t;
	typedef unsigned long int uint32_t;
#endif
typedef float float32_t;
typedef double float64_t; 

enum True_Fals_Type
{
	false=0,
	true=1
};
/*--------------------EMUM--------------------------------*/


#endif

