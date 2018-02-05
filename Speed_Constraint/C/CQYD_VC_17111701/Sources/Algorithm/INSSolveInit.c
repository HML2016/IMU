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

#include "INSSolve.h"


void INS_Load_Data(void){
  

  /*load the temperature value*/

  INS_Sensor_Par[Gyro].data[GyroX] = Sensor_Original_Info.AD_DATA[GX_ADC_CH] / 57.3;
  INS_Sensor_Par[Gyro].data[GyroY] = Sensor_Original_Info.AD_DATA[GY_ADC_CH] / 57.3;
  INS_Sensor_Par[Gyro].data[GyroZ] = Sensor_Original_Info.AD_DATA[GZ_ADC_CH] / 57.3;

  INS_Sensor_Par[Acc].data[AccX] = Sensor_Original_Info.AD_DATA[AX_ADC_CH];
  INS_Sensor_Par[Acc].data[AccY] = Sensor_Original_Info.AD_DATA[AX_ADC_CH];
  INS_Sensor_Par[Acc].data[AccZ] = Sensor_Original_Info.AD_DATA[AX_ADC_CH];

  INS_Sensor_Par[Mag].data[MagX] = Sensor_Original_Info.AD_DATA[MX_ADC_CH];
  INS_Sensor_Par[Mag].data[MagY] = Sensor_Original_Info.AD_DATA[MX_ADC_CH];
  INS_Sensor_Par[Mag].data[MagZ] = Sensor_Original_Info.AD_DATA[MX_ADC_CH];
}

void INSSolve_Init(void){
  INS_Cal_Par.BufferSize = 10;
//  INS_Nav_Par.AlignData  = Command_Par_Info[COMMAND_ALLIGN].Buffer;
}
void INS_Cal(void){
	INS_Nav_Par.AttResol(Bika, 2,
	                     INS_Sensor_Par[Gyro].data, INS_Cal_Par.dt, INS_Nav_Par.Q, INS_Nav_Par.Cbn, INS_Nav_Par.Att);
	INS_Nav_Par.VelResol(INS_Nav_Par.Cbn, INS_Sensor_Par[Acc].data, INS_Nav_Par.Vel, INS_Cal_Par.dt);
	INS_Nav_Par.PosResol(INS_Nav_Par.Vel, INS_Nav_Par.Pos, INS_Cal_Par.dt);
}
