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
#include "TaskSequence.h"

Task_Func_Info Task_Functions[TASK_NUM];

void TaskSequence_Init(void){
  /**/
  Task_Functions[TASK_SENSOR_READ].id           = TASK_SENSOR_READ;
  Task_Functions[TASK_SENSOR_READ].StateType   = Task_Over;
  Task_Functions[TASK_SENSOR_READ].func_p       = Sensor_Original_Info.updata_func;
  Task_Functions[TASK_SENSOR_READ].RunType      = TaskRunInCondition;

//  Task_Functions[TASK_DEV_CALI].id         = TASK_DEV_CALI;
//  Task_Functions[TASK_DEV_CALI].StateType = Task_Over;
//  Task_Functions[TASK_DEV_CALI].func_p     = &Dev_Compensation;
//  Task_Functions[TASK_DEV_CALI].RunType    = TaskRunInCondition;

//  Task_Functions[TASK_TRANSMIT_DATA].id         = TASK_TRANSMIT_DATA;
//  Task_Functions[TASK_TRANSMIT_DATA].StateType = Task_Over;
//  Task_Functions[TASK_TRANSMIT_DATA].func_p     = &Transmit_Data_To_PC; 
//  Task_Functions[TASK_TRANSMIT_DATA].RunType    = TaskRunInCondition;

//  Task_Functions[TASK_COMMAND_RESOLVE].id        = TASK_COMMAND_RESOLVE;
//  Task_Functions[TASK_COMMAND_RESOLVE].StateType= Task_Over;
//  Task_Functions[TASK_COMMAND_RESOLVE].func_p    = &Command_Resolve;
//  Task_Functions[TASK_COMMAND_RESOLVE].RunType   = TaskRunInCondition;

//  Task_Functions[TASK_COMMAND_SEND].id          = TASK_COMMAND_SEND;
//  Task_Functions[TASK_COMMAND_SEND].StateType   = Task_Over;
//  Task_Functions[TASK_COMMAND_SEND].func_p      = &Transmit_Ask_To_PC;
//  Task_Functions[TASK_COMMAND_SEND].RunType      = TaskRunInCondition;

  Task_Functions[TASK_INS_INIT].id             = TASK_INS_INIT;
  Task_Functions[TASK_INS_INIT].StateType     = Task_Ready;
  Task_Functions[TASK_INS_INIT].func_p         = INS_Nav_Par.Init;
  Task_Functions[TASK_INS_INIT].RunType        = TaskRunOnceTime;

  Task_Functions[TASK_MATRIX_INIT].id             = TASK_MATRIX_INIT;
  Task_Functions[TASK_MATRIX_INIT].StateType      = Task_Ready;
  Task_Functions[TASK_MATRIX_INIT].func_p         = Mat_Cal_Info.Init;
  Task_Functions[TASK_MATRIX_INIT].RunType        = TaskRunOnceTime;

  Task_Functions[TASK_KALMAN_INIT].id             = TASK_KALMAN_INIT;
  Task_Functions[TASK_KALMAN_INIT].StateType      = Task_Ready;
  Task_Functions[TASK_KALMAN_INIT].func_p         = Kalman_Cal_Func.Init;
  Task_Functions[TASK_KALMAN_INIT].RunType        = TaskRunOnceTime;

  Task_Functions[TASK_PDR_INIT].id             = TASK_PDR_INIT;
  Task_Functions[TASK_PDR_INIT].StateType      = Task_Ready;
  Task_Functions[TASK_PDR_INIT].func_p         = PDR_Cal_Func.Init;
  Task_Functions[TASK_PDR_INIT].RunType        = TaskRunOnceTime;

  Task_Functions[TASK_ALIGN].id             = TASK_ALIGN;
  Task_Functions[TASK_ALIGN].StateType      = Task_Over;
  Task_Functions[TASK_ALIGN].func_p         = INS_Nav_Par.StaAlign;
  Task_Functions[TASK_ALIGN].RunType        = TaskRunInCondition;

  Task_Functions[TASK_INS_LOAD_DATA].id             = TASK_INS_LOAD_DATA;
  Task_Functions[TASK_INS_LOAD_DATA].StateType      = Task_Over;
  Task_Functions[TASK_INS_LOAD_DATA].func_p         = INS_Nav_Par.LoadData;
  Task_Functions[TASK_INS_LOAD_DATA].RunType        = TaskRunInCondition;

  Task_Functions[TASK_INS_BUFFER_DATA].id             = TASK_INS_LOAD_DATA;
  Task_Functions[TASK_INS_BUFFER_DATA].StateType      = Task_Over;
  Task_Functions[TASK_INS_BUFFER_DATA].func_p         = INS_Nav_Par.BufferData;
  Task_Functions[TASK_INS_BUFFER_DATA].RunType        = TaskRunInCondition;

  Task_Functions[TASK_INS_CAL].id             = TASK_INS_CAL;
  Task_Functions[TASK_INS_CAL].StateType      = Task_Over;
  Task_Functions[TASK_INS_CAL].func_p         = INS_Nav_Par.INS_Cal;
  Task_Functions[TASK_INS_CAL].RunType        = TaskRunInCondition;

  Task_Functions[TASK_PDR_CAL].id             = TASK_PDR_CAL;
  Task_Functions[TASK_PDR_CAL].StateType      = Task_Over;
  Task_Functions[TASK_PDR_CAL].func_p         = PDR_Cal_Func.Solve;
  Task_Functions[TASK_PDR_CAL].RunType        = TaskRunInCondition;
  
//  Task_Functions[TASK_TRANSIT_DEV].id         =TASK_TRANSIT_DEV;
//  Task_Functions[TASK_TRANSIT_DEV].StateType    = Task_Over;
//  Task_Functions[TASK_TRANSIT_DEV].func_p         = &Transmit_Data_To_Dev;
//  Task_Functions[TASK_TRANSIT_DEV].RunType        = TaskRunInCondition;

  Task_Seque_Par.Create(TASK_INS_INIT);
  Task_Seque_Par.Create(TASK_MATRIX_INIT);
  Task_Seque_Par.Create(TASK_KALMAN_INIT);
  
  Task_Seque_Par.Create(TASK_SENSOR_READ);
//  Task_Seque_Par.Create(TASK_DEV_CALI);
  Task_Seque_Par.Create(TASK_INS_LOAD_DATA);
  Task_Seque_Par.Create(TASK_INS_BUFFER_DATA);
  Task_Seque_Par.Create(TASK_ALIGN);
  Task_Seque_Par.Create(TASK_INS_CAL);

//  Task_Seque_Par.Create(TASK_COMMAND_RESOLVE);
//  Task_Seque_Par.Create(TASK_TRANSMIT_DATA);
//  Task_Seque_Par.Create(TASK_TRANSIT_DEV);

  Task_Seque_Par.Activate(TASK_INS_INIT);
  Task_Seque_Par.Activate(TASK_MATRIX_INIT);
  Task_Seque_Par.Activate(TASK_KALMAN_INIT);
//  Task_Seque_Par.Activate(TASK_PDR_INIT);


}

