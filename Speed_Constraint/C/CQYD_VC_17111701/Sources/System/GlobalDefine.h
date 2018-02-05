/***************************************************************** 
*  @brief:       the system global initial, focus on the system set, system work, dev set.
*  @File:        GlobalDefine.h
*  @Project:     HDNT IMU
*  @Author:      liu ning
*  @Date:        20170718
*  @CopyRight:   Copyright (c) 2017 Beijing Information Science & Technology University, 
                                    Beijing Key Laboratory of Hign Dynamic Navigation Technology
                 ISC License (open source)
*  @Version:     17071801
*  @Description: system initial
*****************************************************************/

#ifndef	__GLOBALDEFINE_H__
#define	__GLOBALDEFINE_H__

/*-select the device platform-*/

#if defined(STM32F429xx)
  #include "stm32f4xx_hal.h"
#elif defined(STM32F767xx)
  #include "stm32f7xx_hal.h"
#endif

#if defined(VS_SIM)
  #include "PlatformCon.h"
#else
  #include "Config.h"
  #include "SensorRead.h"
  #include "PlatformCon.h"
  #include "DataStorage.h"
  #include "DataLink.h"
  #include "DevCompensation.h"
  #include "TaskSequence.h"
  #include "SysKeyWord.h"
#endif
#include "INSSolve.h"
#include "KFFilter.h"

#define YEAR ((((__DATE__[7]-'0')*10)+(__DATE__[8]-'0'))*10\
             +(__DATE__[9]-'0')*10+(__DATE__[10]-'0'))      
#define MONTH (__DATE__ [2] == 'n' ? 1\
    : __DATE__ [2] == 'b' ? 2 \
    : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 3 : 4) \
    : __DATE__ [2] == 'y' ? 5 \
    : __DATE__ [2] == 'n' ? 6\
    : __DATE__ [2] == 'l' ? 7\
    : __DATE__ [2] == 'g' ? 8\
    : __DATE__ [2] == 'p' ? 9 \
    : __DATE__ [2] == 't' ? 10 \
    : __DATE__ [2] == 'v' ? 11 : 12)  
  
#define DAY ((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10 \
    + (__DATE__ [5] - '0')) 
    
#define DATE_AS_INT ((YEAR-2000) * 10000 + (MONTH) * 100 + (DAY)) 

#define SW_VER     DATE_AS_INT 

#define PI 3.1415927f
#define RAM_SIZE 300    //used to buffer the calculate about matrix and kalman
#define g 9.8f          //difine the gravity acceleration

enum Run_Mode_Type{
  RunInIMU=0,
  RunInAHRS,
  RunInNav,
  RunInPDR
};

enum Dev_Mode_Type{
  PlaceSingleDev=0,
  PlaceDoubleDev,
  PlaceMultiDev,
};

enum Dev_Cali_Mode_Type{
  CaliRC=0,
  CaliRCTN=1,
  CaliRCTNSF,
  CaliRCTNSFCR,
  CaliPAR
};

enum Protcol_Mode_Type{
  ProtIEEE754=0,    //in basic 754 Protcol
  Prot2D,           //in basic 2 
  ProtBMD           //in define by self
};

typedef struct{
  uint8_t   Run_Mode;        // system run mode see the work report
  
  float32_t Output_SPS;      // output refresh time
  float32_t  Output_Baud;     //
  
  float32_t Integral_C;     //
  
  uint8_t   Dev_Mode;       //
  uint8_t   Cali_Mode;      //
  uint8_t   Cali_Num;       //
  
  uint8_t   Align_Mode;     // Initial alignment mode
  uint8_t   Att_Cal_Mode;   // the attitude caculate mode 0 transfer vector; 1 bika;
  uint8_t   Vector_Sample_Num;   //the rotation vetctor sample number
  
  uint8_t   Output_Prot_Mode;
  uint8_t   Output_Data_Mode;  //output data mode
  uint8_t   Output_Ch_Num;   //
  float32_t Output_Data[256];  //buffer the system output data
  uint8_t   Output_Ch[256];
  float32_t Send_To_User_Data[256];  //based on the command set, put the user data to the dev.

}Dev_Work_Info;

extern Dev_Work_Info Dev_Set_Info;

typedef struct{
  uint32_t Time_Int;   //system time in uint32_t
  uint32_t Time_Inc;   //inc time
  uint32_t Sensor_Time_Int;
  uint32_t Sensor_Time_Inc;

  float32_t System_Time;    

  float32_t Mes_Time;       

  uint32_t  Product_ID;     //
  uint8_t   Soft_Version;   //

}System_State_Info;

extern System_State_Info System_Work_Info;

void GlobalDefine_Init(void);
void Sys_Soft_Reset(void);
#endif
