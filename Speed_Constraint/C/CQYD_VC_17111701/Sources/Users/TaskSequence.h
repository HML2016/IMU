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
#ifndef __TASKSEQUENCE_H
#define __TASKSEQUENCE_H
/*-select the device platform-*/
#if defined(STM32F429xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F767xx)
  #include "stm32f7xx_hal.h"
#endif
#include "PlatformCon.h"
#include "SensorRead.h"
#include "DevCompensation.h"
#include "GlobalDefine.h"
#include "TransmitData.h"
#include "DataLink.h"
#include "PDRSolve.h"

/*******************************************************
*******************************************************/
#define TASK_SENSOR_READ     0   //the sensor sample

#define TASK_DEV_CALI        1   //device calibration

#define TASK_DATA_ANALY      2

#define TASK_GNSS_SOL        3

#define TASK_TRANSMIT_DATA   4   //transmit the data to pc
#define TASK_COMMAND_RESOLVE 5
#define TASK_COMMAND_SEND    6

#define TASK_INS_INIT        7   //algorithm initial
#define TASK_MATRIX_INIT     8
#define TASK_KALMAN_INIT     9
#define TASK_PDR_INIT        10

#define TASK_ALIGN           11
#define TASK_INS_LOAD_DATA   12
#define TASK_INS_BUFFER_DATA 13
#define TASK_INS_CAL         14
#define TASK_PDR_CAL         15

#define TASK_MAG_RESET       16

#define TASK_POWER_CON       17

#define TASK_TRANSIT_DEV     18

#define TASK_SEQ_STO         19
#define TASK_SET             20
#define TASK_CLEAR_SEQUENCE  21
#define TASK_NUM             TASK_CLEAR_SEQUENCE+1
#define MAX_TASK_NUM         TASK_NUM*2

enum Task_State_Type{
  Task_Working=0,   //is working 
  Task_Over,        //work over
  Task_Ready,
  Task_Suspend,     //suspend
};

enum Task_Run_Type{
  TaskRunForever=0,
  TaskRunInCondition,
  TaskRunOnceTime,
  TaskRunNeverRun
};

typedef struct{
  uint8_t id;
  void (*func_p) (void);
  uint8_t StateType;
  uint8_t RunType;
  uint8_t Priority;
}Task_Func_Info;

extern Task_Func_Info Task_Functions[TASK_NUM];
extern uint16_t TaskSequeceData[MAX_TASK_NUM];
extern uint16_t Current_Task_Num;

typedef struct{
  uint16_t TaskSque[MAX_TASK_NUM];
  uint16_t TaskNum;
  void (*Create) (uint16_t);
  void (*Delete) (uint16_t);
  void (*Activate) (uint16_t);
  uint16_t CurrentTask;
}Task_Manage_Info;

extern Task_Manage_Info Task_Seque_Par;


void TaskSequence_Init(void); 
void Task_Run_Sequence(void);

#endif
