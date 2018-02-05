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
#ifndef __TRANSMITDATA_H
#define __TRANSMITDATA_H

#include "GlobalDefine.h"

void Load_Data(void);             //load the temp data
void Transmit_Data_To_PC(void);  //Transmit the Data to PC used to control this device
void Transmit_Ask_To_PC(void);
void Transmit_Data_To_Dev(void);//Transmit the Data to other device, for example: BT or GNSS

#endif


