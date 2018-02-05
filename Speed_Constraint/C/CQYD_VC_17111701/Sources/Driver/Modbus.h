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
#ifndef __MODBUS_H
#define __MODBUS_H

#include "GlobalDefine.h"
/* CRC low byte check*/
typedef struct{
  uint8_t StaAdd;   //define the station address.
  uint8_t FuncCode;
  uint8_t DataLength;
  float32_t Data[100];
  uint8_t *BuffData;
}Modbus_Data;

typedef struct{
  Modbus_Data *Info;
  uint16_t (*BuffGen) (Modbus_Data *);
  uint16_t (*Crc) (uint8_t *, uint16_t);
}Modbus_Sol_Info;

extern Modbus_Sol_Info Modbus_Solve;

#endif

