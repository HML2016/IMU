/**/
#include "PDRSolve.h"

PDR_Info PDR_Set_Par = { 0, true, true, 0,0.01f, 0.1f*ATR, 0.3E5f, 3 ,0,0};
PDR_Func_Info PDR_Cal_Func = { &PDRSolve_Init, &PDR_Detector, &PDR_Cal};

static float32_t Tk=0;
static float32_t StepFlag = false;

vec3 temp1={0};
float32_t temp2=0,gyro2=0;

void PDR_Detector(void){
/* This must add the velocity constraint, if have v, the zupt=true
  if(){
    PDR_Set_Par.Zupt=true;

  }else{
    PDR_Set_Par.Zupt=false;
	  StepFlag = false;
  }
  */
}

