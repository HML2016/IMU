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
#ifndef	__GNSSSOLVE_H__
#define	__GNSSSOLVE_H__
#if defined(STM32F429xx)
   #include "stm32f4xx_hal.h"
#endif
#include "PlatformCon.h"
#include "math.h"
#include "GlobalDefine.h"

#ifdef GNSS_NJUST_MBT
	#define GPS_LENGTH 94
#elif GNSS_0183

#endif
typedef struct
{
	uint32_t DaySecond;    //Day Second
	uint32_t WeekSecond;   //Week Second
	uint32_t NavType;      //Position Mark
	float64_t DesX;
	float64_t DesY;
	float64_t DesZ;
	float64_t Lat;
	float64_t Lon;
	float32_t high;
	float32_t Vx;
	float32_t Vy;
	float32_t Vz;
	float32_t Ve;
	float32_t Vn;
	float32_t Vu;
	uint8_t SatNum;
	uint8_t PDOP;
	uint32_t FirTime;
}GNSS;
extern GNSS GNSSInfo;

typedef struct
{
	float32_t UTCTime;
	uint32_t NavType;
	float32_t DesX;
	float32_t DesY;
	float32_t DesZ;
	float32_t Lat;
	float32_t Lon;
	float32_t high;
	float32_t Vx;
	float32_t Vy;
	float32_t Vz;
	float32_t Ve;
	float32_t Vn;
	float32_t Vu;
	uint8_t SatNum;
	uint8_t PDOP;
	uint8_t Lo_EW;   //East and West Longitude
	uint8_t La_SN;
	float32_t Angle_T;
	float32_t V_MCN;
	float32_t Angle_MCT;
	float32_t Angle_M;
	float32_t V_N;
	float32_t V_K;
	
}GNSSAdvace;
extern GNSSAdvace GNSSAdvInfo;

/*----------------------Function Declaration------------------------------*/
void GPS_MBT_Resolve(uint8_t *buf,uint16_t num);    //BMD Project GPS, HW
void GPS_0183_Resolve(uint8_t *buf,uint16_t data_num);   //0183 protol GPS
 
#endif
