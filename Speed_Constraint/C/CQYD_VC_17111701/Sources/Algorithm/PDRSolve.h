/**/
#ifndef __PDRSOLVE_H
#define __PDRSOLVE_H

#include "PlatformCon.h"
#include "INSSolve.h"
#include "Matrix.h"
#include "KFFilter.h"


typedef struct{
  uint8_t Mode;
  uint8_t Zupt;
  uint8_t Zaru;
  uint32_t StepNum;

  float32_t Sigma_a;
  float32_t Sigma_g;
  float32_t Gamma;
  float32_t WindowSize;
  float32_t *BuffData;
    
  float32_t Tk;
}PDR_Info;

typedef struct{
	void(*Init) (void);
	void(*Detector)(void);
	void(*Solve) (void);
}PDR_Func_Info;

extern PDR_Info PDR_Set_Par;
extern PDR_Func_Info PDR_Cal_Func;

void PDR_Detector(void);
void PDRSolve_Init(void);
void PDR_Cal(void);
#endif



