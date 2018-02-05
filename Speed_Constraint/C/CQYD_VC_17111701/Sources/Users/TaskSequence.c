/***************************************************************** 
*  @brief:         
*  @File:          
*  @Project:     
*  @Author:      
*  @Date:         
*  @CopyRight:    copyright Copyright (c) 2017 HDNT, ISC License (open source)
*  @Version:     
*  @Description:
*****************************************************************/
#include "TaskSequence.h"

void Task_Create(uint16_t id);
void Task_Delete(uint16_t id);
void Task_Activate(uint16_t id);


Task_Manage_Info Task_Seque_Par={{0},0,&Task_Create,&Task_Delete,&Task_Activate};

void Task_Run_Sequence(void){
  uint16_t i=0;
  for(i=0;i<Task_Seque_Par.TaskNum;i++){
    Task_Seque_Par.CurrentTask=i;
    switch(Task_Functions[Task_Seque_Par.TaskSque[i]].RunType){
      case (TaskRunForever):{
        Task_Functions[Task_Seque_Par.TaskSque[i]].StateType=Task_Working;
        Task_Functions[Task_Seque_Par.TaskSque[i]].func_p();
        Task_Functions[Task_Seque_Par.TaskSque[i]].StateType=Task_Over;
        break;
      };
      case (TaskRunInCondition):{
        if(Task_Functions[Task_Seque_Par.TaskSque[i]].StateType==Task_Ready){
          Task_Functions[Task_Seque_Par.TaskSque[i]].StateType=Task_Working;
          Task_Functions[Task_Seque_Par.TaskSque[i]].func_p();
          Task_Functions[Task_Seque_Par.TaskSque[i]].StateType=Task_Over;
        }
        break;
      };
      case(TaskRunOnceTime):{
        if(Task_Functions[Task_Seque_Par.TaskSque[i]].StateType==Task_Ready){
          Task_Functions[Task_Seque_Par.TaskSque[i]].StateType=Task_Working;
          Task_Functions[Task_Seque_Par.TaskSque[i]].func_p();
          Task_Functions[Task_Seque_Par.TaskSque[i]].StateType=Task_Suspend;
          Task_Seque_Par.Delete(Task_Seque_Par.TaskSque[i]);
        }
        break;
      };
    };
  }
  
}
/***************************************************************** 
*  @brief:          
*  @Function: 
*  @inparam:
*  @outparam:  
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:  creat the task
*****************************************************************/
void Task_Create(uint16_t id){
  Task_Seque_Par.TaskSque[Task_Seque_Par.TaskNum]=id;
  Task_Seque_Par.TaskNum++;
}
/* not finish */
void Task_Preem(uint16_t id, uint8_t priority){
  uint16_t i=0;
  if(Task_Seque_Par.CurrentTask==Task_Seque_Par.TaskNum-1){
    
  }else{
    for(i=Task_Seque_Par.CurrentTask+1;i<Task_Seque_Par.TaskNum;i++){
      Task_Seque_Par.TaskSque[i+1]=Task_Seque_Par.TaskSque[i];
    }
    Task_Seque_Par.TaskSque[Task_Seque_Par.CurrentTask+1]=id;
  }
}
void Task_Delete(uint16_t id){
  uint16_t i=0;
  uint16_t TaskNum=Task_Seque_Par.TaskNum;
  Task_Seque_Par.TaskNum=0;
  for(i=0;i<TaskNum;i++){
    if(Task_Seque_Par.TaskSque[i]==id){
      
    }else{
      Task_Seque_Par.TaskSque[Task_Seque_Par.TaskNum]=Task_Seque_Par.TaskSque[i];
      Task_Seque_Par.TaskNum++;
    }
  }
}
void Task_Activate(uint16_t id){  //activate the runincondition task
  if(Task_Functions[id].StateType!=Task_Ready){
    Task_Functions[id].StateType=Task_Ready;
  }
}


