
/***************************************************************** 
*  @brief:          
*  @Function:   
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
#ifndef __BAROREAD_H
#define __BAROREAD_H

#include "PlatformCon.h"
#include "IMUConfig.h"
#include "Delay.h"
#include "myiic.h"
#include "math.h"

typedef uint8_t  u8;

#define MS5611_ADDR          0xEE     // default I2C address

// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00  //Conversion time 0.6ms  Resolution 0.065mbar
#define MS561101BA_OSR_512 0x02  //Conversion time 1.2ms  Resolution 0.042mbar
#define MS561101BA_OSR_1024 0x04 //Conversion time 2.3ms  Resolution 0.027mbar
#define MS561101BA_OSR_2048 0x06 //Conversion time 4.6ms  Resolution 0.018mbar
#define MS561101BA_OSR_4096 0x08 //Conversion time 9.1ms  Resolution 0.012mbar

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.

//Other
#define MSLP                    101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)

/*压强计控制*/
//#define BARO_CSB_CL  GPIO_ResetBits(GPIOE,GPIO_Pin_4)  //压强计片选信号
#define BARO_CSB_CL  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET)

//#define BARO_CSB_CH  GPIO_SetBits(GPIOE,GPIO_Pin_4)
#define BARO_CSB_CH  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET)

//#define BARO_PS_CL  GPIO_ResetBits(GPIOE,GPIO_Pin_3)  //压强计片选信号
#define BARO_PS_CL  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET)

//#define BARO_PS_CH  GPIO_SetBits(GPIOE,GPIO_Pin_3)
#define BARO_PS_CH  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET)

extern uint8_t BARO_Reset_Flag;
//  Temperature in 0.01C
//  Pressure    in 0.01mbar = Pa
//  Altitude    in meter  / cm
extern float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude;
extern float ALT_Update_Interval;
extern uint8_t ALT_Updated ; //????????????

void MS561101BA_init(void);
void MS5611BA_Routing(void);
void MS561101BA_ResetAlt(void);
float MS5611BA_Get_D(void);
void MS561101BA_SetAlt(float Current);
void MS561101BA_reset(void);
void MS561101BA_readPROM(void);
void MS561101BA_NewTemp(float val);
void MS561101BA_NewPress(float val);
void MS561101BA_NewAlt(float val);
float MS561101BA_getAvg(float * buff, int size);
void MS561101BA_startConversion(uint8_t command);
unsigned long MS561101BA_getConversion(void);
void MS561101BA_GetTemperature(void);
float MS561101BA_get_altitude(void);
void MS561101BA_getPressure(void);

#endif
