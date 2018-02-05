/***************************************************************** 
*  @brief:          
*  @Function: 
*  @inparam:
*  @outparam:  
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
#ifndef  __DATALINK_H
#define  __DATALINK_H

/*-select the device platform-*/
#if defined(STM32F429xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F767xx)
  #include "stm32f7xx_hal.h"
#endif
#include "PlatformCOn.h"
#include "GlobalDefine.h"
#include "DataStorage.h"
#include "SensorRead.h"
#include "DataStorage.h"
//#include "DevCompensation.h"
#include "Modbus.h"

#define COMMAND_SID  0

#define COMMAND_SETTING  0

#define COMMAND_DEV_RC   1
#define COMMAND_DEV_TN   2
#define COMMAND_DEV_SF   3
#define COMMAND_DEV_CR   4
#define COMMAND_DEV_TSF  5
#define COMMAND_DEV_GA   6    //
#define COMMAND_DEV_PAR  7    //attach parameter command

#define COMMAND_FIL_CH1  8
#define COMMAND_FIL_CH2  9
#define COMMAND_FIL_CH3  10
#define COMMAND_FIL_CH4  11
#define COMMAND_FIL_CH5  12
#define COMMAND_FIL_CH6  13
#define COMMAND_FIL_CH7  14
#define COMMAND_FIL_CH8  15
#define COMMAND_FIL_CH9  16
#define COMMAND_FIL_CH10  17
#define COMMAND_FIL_CH11  18
#define COMMAND_FIL_CH12  19
#define COMMAND_FIL_CH13  20
#define COMMAND_FIL_CH14  21
#define COMMAND_FIL_CH15  22

#define COMMAND_KALMAN   23   //KALMAN filter par

#define COMMAND_DEVCON   24   //device control
#define COMMAND_USER     25   //user data, for example: pdr, ir, mag

#define COMMAND_RES      26   //reserved
#define COMMAND_RES1     27   //reserved1

#define COMMAND_IQL_CALI   28   //inquire the calibration data
#define COMMAND_IQL_STATUS 29   //inquire the device status

#define COMMAND_ALLIGN   30   //allignment 

#define COMMAND_COMMU    31  //communication with other device

#define COMMAND_FLASH_OP 32   //control the flash operation

#define COMMAND_NUM      COMMAND_FLASH_OP+1   //the flash command is the last command

#define HEADER_LENGTH    2

#define RUN_MODE_ADD      5
#define DEV_MODE_ADD     18
#define CALI_MODE_ADD    19
#define CALI_NUM_ADD     20
#define ALIGN_MODE_ADD   21
#define ATT_CAL_MODE_ADD 22
#define VECTOR_SAMPLE_ADD 23
#define OUTPUT_PROT_ADD  24
#define OUTPUT_DATA_ADD  25
#define OUTPUT_CH_NUM_ADD    26

#define Mes_Data_ADD      6

/*ensure the command valid*/
enum Command_Flag_Info{
  Command_Valid=0, 
  Command_InValid=1
};

enum Command_Ask_Flag{
  Ask_Valid=0,
  Ask_InValid
};

typedef struct{
  uint8_t   Header[HEADER_LENGTH];   //the command header
  uint8_t   Data[1024];               //the command data
  float32_t Buffer[256];
  uint16_t  Length;                  //the length
  uint8_t   Type;
  uint8_t   StorageFlag;             //storage flash flag 0 is not storage
  uint8_t   Flag;                    //run flag 
  uint32_t  Sector;

  void (*Resovle_Func) (void);
  void (*Storage_Func) (void);
  
  uint8_t   id;
  uint8_t   Ask_Flag;              //this command wether the ask command 0 n,1 y
  uint32_t  StartAdd;              //Start address in the par_setting_sector
  uint32_t  FlagAdd;               //StartAdd storage add
  
}Command_Info;

extern Command_Info Command_Par_Info[COMMAND_NUM];

extern Command_Info *Current_Command_Info;
/*
union TransData{
  float32_t buffer[]
};*/

#define OUTPUT_COMMAND_NUM 6

#define REP_COMMAND           0
#define REP_COMMAND_STATUS    1
#define REP_COMMAND_CALI      2
#define REP_OUTPUT            3

typedef struct{
  uint8_t Header[2];
}Output_Command_Info;

extern Output_Command_Info Output_Command_Par[OUTPUT_COMMAND_NUM];

/*system setting command resolve*/
void Command_Setting_Resolve(void);
/*command calibration resovle */
void Command_DEV_TN_Resolve(void); 
void Command_DEV_CR_Resolve(void);
void Command_DEV_RC_Resolve(void);
void Command_DEV_GA_Resolve(void);
void Command_DEV_PAR_Resolve(void);
void Command_DEV_SF_Resolve(void);
void Command_DEV_TSF_Resolve(void);
/*allignment command inlcude transfer and initial*/
void Command_Allign_Resolve(void);
/*external device control*/
void Command_DevCon_Resolve(void);
/*flash operation control*/
void Command_Flash_Resolve(void);
/*fir filter command*/
void Command_FIR_CH1_Resolve(void);
void Command_FIR_CH2_Resolve(void);
void Command_FIR_CH3_Resolve(void);
void Command_FIR_CH4_Resolve(void);
void Command_FIR_CH5_Resolve(void);
void Command_FIR_CH6_Resolve(void);
void Command_FIR_CH7_Resolve(void);
void Command_FIR_CH8_Resolve(void);
void Command_FIR_CH9_Resolve(void);
void Command_FIR_CH10_Resolve(void);
void Command_FIR_CH11_Resolve(void);
void Command_FIR_CH12_Resolve(void);
void Command_FIR_CH13_Resolve(void);
void Command_FIR_CH14_Resolve(void);
void Command_FIR_CH15_Resolve(void);
/*kalman paramter command*/
void Command_Kalman_Resolve(void);
/*external device control command*/
void Command_DevCon_Resolve(void);
void Command_User_Resolve(void);
/*Resovle Command*/
void Command_Res_Resolve(void);
void Command_Res1_Resolve(void);

void Command_Iqiure_CALI_Resolve(void);
void Command_Iqiure_Status_Resolve(void);
void Command_Commu_Resovle(void);

void Command_PC_ASK_Resolve(void);
void Command_Resolve(void);

/*command receive ask*/
uint16_t Command_ASK_Send(uint8_t * buffer, uint8_t * header, uint16_t length);
uint16_t Command_Output_Send(uint8_t *buffer,uint8_t UartNum,uint8_t Protocol);
uint16_t Command_Commu_Send(uint8_t *buffer);
uint16_t Command_Answer_Send(uint8_t *buffer);
uint16_t Command_Modbus_Send(float32_t *Data,uint8_t *buffer,uint16_t Size);
void DataLink_Init(void);
#endif


