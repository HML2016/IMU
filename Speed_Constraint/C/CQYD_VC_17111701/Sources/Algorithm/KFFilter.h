/**/
#ifndef __KFFILTER_H
#define __KFFILTER_H

#include "PlatformCon.h"
#include "INSSolve.h"
#include "Matrix.h"

#define DIM_STATE  9    //define the state dimension (X),DS
#define DIM_OBS    3    //DO
#define DIM_Proc   6    //DP  process noise dimension

typedef float32_t matDSxDS[DIM_STATE*DIM_STATE];
typedef float32_t matDPxDP[DIM_Proc*DIM_Proc];
typedef float32_t matDOxDO[DIM_OBS*DIM_OBS];
typedef float32_t matDOxDS[DIM_OBS*DIM_STATE];
typedef float32_t matDSxDO[DIM_STATE*DIM_OBS];
typedef float32_t matDSxDP[DIM_STATE*DIM_Proc];
typedef float32_t vecDO[DIM_OBS];
typedef float32_t vecDS[DIM_STATE];
typedef float32_t matDPxDS[DIM_Proc*DIM_STATE];
/*typedef float32_t vec33[3];*/
typedef float32_t vec3[3];


typedef struct{
  matDSxDS P;  //covariance matrix
  matDPxDP Q;  //process noise
  matDOxDO R;  //measurement noise matrix
  matDOxDS H;  //covariance matrix
  vecDO    Z;
  vecDS    X;
  vecDS    dX;
  matDSxDO K;
  matDSxDS F;
  matDSxDP G;
  matDSxDS Id;
}Kalman_Info;

extern Kalman_Info Kalman_Par;


typedef struct {
  vec3 SigmaAcc;    //% Process noise for modeling the accelerometer noise (x,y,z platform 
                    //% coordinate axis) and other accelerometer errors [m/s^2].
  vec3 SigmaGyro;
  vec3 AccDrNoise;
  vec3 GyroDrNoise;
  vec3 SigmaVel;
  vec3 SigmaInPos;
  vec3 SigmaInVel;
  vec3 SigmaInAtt;
  vec3 SigmaInAccBias;
  vec3 SigmaInGyroBias;
  vec3 SigmaInAccScale;
  vec3 SigmaInGyroScale;
  float32_t AcctTimeCon;
  float32_t GyroTimeCon;
  float32_t *BuffData;
}Kalman_Set_Info;

extern Kalman_Set_Info Kalman_Set_Par;

typedef struct{
	void(*Init) (void);
	void(*StateCal)(vec3, vec3);
	void(*StateUpdate)(void);
	void(*TimeUpdate)(void);
	void(*MeasureUpdate)(void);
}Kalman_Cal_Info;

extern Kalman_Cal_Info Kalman_Cal_Func;

/*functions*/

void KFFilter_Init(void);
void KFFilter_Cal_State(vec3 GyroData, vec3 AccData);
void KFFilter_State_Update(void);
void KFFilter_Time_Updata(void);
void KFFilter_Measure_Update(void);

#endif

