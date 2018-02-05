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
#ifndef __INSSOLVE_H
#define __INSSOLVE_H

#include "PlatformCon.h"
#include "IMUConfig.h"
#include "Matrix.h"
#include "GlobalDefine.h"


#if defined(VS_SIM)
#include "Algorithm_Main.h"
#endif


#define RTA 180/PI

#define ATR PI/180

#define espp 0.0000001f

typedef float32_t mat3x3[3 * 3];  //typedef the 3*3 dimention matrix, the row is 3 and the column is 3
typedef float32_t vec3[3];        //the column vector
typedef float32_t mat3x9[3 * 9];
typedef float32_t vec4[4];
typedef float32_t vec6[6];
typedef float32_t vec10[10];

/*Sensor buffer set*/
#define INERTIAL_MAX_BUFFER_SIZE 100   //record the sensor buffer length used to caculate information

enum Att_Resolve_Type{
  Bika=0,
  RotationVector
};

enum Init_Align_Type{
  AlignAcc=0,
  AlignAccMag,
  AlignTrans,
  AlignComplex,
  AlignMotion
};

typedef struct{
  float32_t data[AXIS_NUM];
  float32_t BufferData[AXIS_NUM][INERTIAL_MAX_BUFFER_SIZE];
  float32_t Mean[AXIS_NUM];
}Inertial_Sensor_Info;

extern Inertial_Sensor_Info INS_Sensor_Par[DEV_TYPE_NUM];

typedef struct{
  float32_t data;
  float32_t BufferData[INERTIAL_MAX_BUFFER_SIZE];
}Baro_Sensor_Info;

typedef struct{
  float32_t *AlignData;
  mat3x3 Cbn;
  vec4 Q;
  vec3 Att;
  vec3 Pos;
  vec3 Vel;
  void (*Init) (void);
  void (*StaAlign) (void);
  void (*LoadData) (void);
  void (*BufferData) (void);
  void (*AttResol) (uint8_t, uint8_t, vec3, vec10,vec4, mat3x3, vec3);
  void (*VelResol) (mat3x3, vec3, vec3, vec10);
  void (*PosResol) (vec3, vec3, vec10);
  void(*INS_Cal) (void);
  float32_t(*ReadTime) (void);
  void (*q2dcm) (vec4, mat3x3);
  void (*dcm2e) (mat3x3,vec3);
  void (*dcm2q) (mat3x3,vec4);
  void (*e2dcm) (vec4,mat3x3);
}Nav_Info;

extern Nav_Info INS_Nav_Par;

#define MAX_SAMPLE_NUM 10

typedef struct{
  vec10 dt;   //refresh time
  float32_t BufferSize;   //buffer size of inertial size
}INS_Cal_Info;
extern INS_Cal_Info INS_Cal_Par;

void INS_Load_Data(void);
void INSSolve_Init(void);
void INS_Cal(void);

#endif


