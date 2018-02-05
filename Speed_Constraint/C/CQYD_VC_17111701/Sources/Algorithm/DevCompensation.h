/***************************************************************** 
*  @brief:     Error Compensation about gyro acc mag and baro and so on.
*  @File:      DevCompensation
*  @Project:   MIMU
*  @Author:    Liu Ning
*  @Date:      170603
*  @CopyRight: HDNT @ Copyright 2017
*  @Version:   V1.0
*  @Description:
*****************************************************************/
#ifndef	__DEVCOMPENSATION_H__
#define	__DEVCOMPENSATION_H__
/*-----------------------Import Plat Information----------------------------*/
#include "IMUConfig.h"
#include "PlatformCon.h"
#include "DataLink.h"
/*-----------------------Global Parameters Declaration-----------------------------*/

/*this struct is the dev description*/
typedef struct{
  float32_t  Raw_Data;
  float32_t RC_Data;
  float32_t TN_Data;
  float32_t SF_Data;
  float32_t CR_Data;
  float32_t TSF_Data;
  float32_t GA_Data;
  float32_t PAR_Data;
  float32_t Temp_Data;   //temperature value
  float32_t DataBuffer[200];  //the max fir filter order is 200
  int16_t BufferSize;
  float32_t FirFlag;
  float32_t Out_Data;    //output data to the caculate
}Dev_Info;

extern Dev_Info Dev_Sol_Info[DEV_NUM];

/*descritption the error model state*/
typedef struct{
  float32_t *RC_Data;
  float32_t *TN_Data;
  float32_t *SF_Data;
  float32_t *CR_Data;
  float32_t *TSF_Data;
  float32_t *GA_Data;
  float32_t *PAR_Data;
}Error_Info;

extern Error_Info Dev_Err_Info;

#define FIR_CHNUM 15

typedef struct{
  uint16_t FirOrder;
  float32_t *Coe;
}FIR_Info;

extern FIR_Info Fir_Par[FIR_CHNUM];

#define CR_NUM 2
extern const uint8_t CR_Coe[DEV_NUM][CR_NUM];
/*every device includes parameter number*/
#define RC_PAR_NUM 2
#define TN_PAR_NUM 3
#define SF_PAR_NUM 3
#define CR_PAR_NUM 6
#define TSF_PAR_NUM 3
#define GA_PAR_NUM 3
#define PAR_NUM 20

void Dev_Compensation(void);
void DevCompensation_Init(void);
void Load_Dev_Data(void);


#endif


