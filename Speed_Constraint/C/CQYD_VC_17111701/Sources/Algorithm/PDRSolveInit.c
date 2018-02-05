/**/
#include "PDRsolve.h"

void PDRSolve_Init(void){

}

void PDR_Cal(void){

	INS_Cal_Par.BufferSize = PDR_Set_Par.WindowSize;

	Kalman_Cal_Func.StateCal(INS_Sensor_Par[Gyro].data, INS_Sensor_Par[Acc].data);
	Kalman_Cal_Func.TimeUpdate();
	PDR_Cal_Func.Detector();   //add the velocity constraint
	if (PDR_Set_Par.Zupt == true){
		Kalman_Cal_Func.MeasureUpdate();
		Kalman_Cal_Func.StateUpdate();
	}
}

