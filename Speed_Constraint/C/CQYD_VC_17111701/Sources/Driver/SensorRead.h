/***************************************************************** 
*  @brief:       Sensor Input Through External ADC/ Internal ADC/FMC  
*  @File:        SensorRead.h
*  @Project:     MIMU
*  @Author:      Liu Ning
*  @Date:        20170611 
*  @CopyRight:   Copyright (c) 2017 HDNT, ISC License (open source)
*  @Version:     V1.0
*  @Description: SensorRead header file
*****************************************************************/

#ifndef __SENSORREAD_H
#define __SENSORREAD_H

#if defined(STM32F429xx)
   #include "stm32f4xx_hal.h"
#endif

#include "PlatformCon.h"
#include "IMUConfig.h"
#include "Delay.h"
#include "mpu9250.h"
#include "BaroRead.h"

/*-AD channel-*/
#define GX_ADC_CH   4
#define GY_ADC_CH   5
#define GZ_ADC_CH   6
#define AX_ADC_CH   0
#define AY_ADC_CH   1
#define AZ_ADC_CH   2
#define MX_ADC_CH   7
#define MY_ADC_CH   8
#define MZ_ADC_CH   9
#define TEMP_ADC_CH 3

/*-Global Variable Declarition-*/
#define FIR_CH     88
/*Sensor's struct used to descript the sensor's character.*/

typedef struct{
  float32_t AD_DATA[FIR_CH];            //storage adc data from adc
  float32_t AD_Buffer[FIR_CH][256];     //the max order is 255
#if defined(AD_FIR_MODE)
  uint8_t  AD_Buffer_Size[FIR_CH];     //adc buffer length, this value will changed through the filter mess command
  uint16_t Fitler_Coe[FIR_CH][256];    //Fir coe array
  uint16_t ADF_DATA[FIR_CH];           //after filter data
#endif
  void (*updata_func) (void);
  uint8_t (*SelfTest_func) (void);
  void (*Init_Func) (void);
  uint8_t FIR_Flag; //
}ADC_Info;

extern ADC_Info Sensor_Original_Info;

void Sensor_Read_Init(void);

void Sensor_Updata(void); //all the sensor's data updata from adc
uint8_t Sensor_Check(void);
void Sensor_Init(void);


///2017.07.27

void SPI_RW(uint8_t byte,uint8_t *output);//byte 写入内容  output读取内容
void MPU9255_ReadReg( uint8_t ReadAddr, uint8_t *ReadData );
void MPU9255_WriteReg( uint8_t WriteAddr, uint8_t WriteData );
void MPU9255_ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes );
void MPU9255_Mag_WriteReg( uint8_t WriteAddr, uint8_t WriteData );
void MPU9255_Mag_ReadReg( uint8_t ReadAddr,uint8_t* ReadBuf );
void MPU9255_Config( void );
uint8_t MPU9255_Check(void);
uint8_t MPU9255_Init( MPU_InitTypeDef *MPUx );
void Sensor_Read(void);
uint8_t MPU9255_Config_Check( void );
	
///2017.07.27
#endif
