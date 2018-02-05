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
#include "SensorDataFilter.h"

uint16_t Filter_Coe[10][20]={0};
/***************************************************************** 
*  @brief:          
*  @Function:  Row is the adc channel number;
               Column is the adc buffer size
							 output is the output array
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void AD_Filter(uint16_t *input,uint8_t Row,uint8_t Column,uint16_t *output){
	uint16_t i=0,j=0;
  for(i=0;i<Row;i++){
		 output[i]=0;
	   for(j=0;j<Column;j++){
			 output[i]=output[i]+input[i*Row+j]*Filter_Coe[i][j];
		 }
	 }
}

