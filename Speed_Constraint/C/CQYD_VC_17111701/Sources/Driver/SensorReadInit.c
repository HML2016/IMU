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
#include "SensorRead.h"
/*-Global variable define-*/
ADC_Info Sensor_Original_Info={{0},{0},&Sensor_Updata,&Sensor_Check,&Sensor_Init,0};

/***************************************************************** 
*  @brief:          
*  @Function:   
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void Sensor_Read_Init(void){
	
  Sensor_Original_Info.Init_Func();

	/*-define the default fir filter parameter-*/
	//Sensor_Original_Info.Fitler_Coe[GX_ADC_CH][256]={0};
}

