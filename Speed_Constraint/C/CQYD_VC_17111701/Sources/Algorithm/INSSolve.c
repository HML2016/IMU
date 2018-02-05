/***************************************************************** 
*  @brief:         
*  @File:          
*  @Project:     
*  @Author:      
*  @Date:       170620
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
#include "INSSolve.h"

/*define the functions*/

float32_t sqrt_hf(float32_t arg);
void quat2rotation(vec4 q, mat3x3 rotmat);   //quatention to rotation matrix
void rotation2euler(mat3x3 rotmat, vec3 euler);
void rotation2quat(mat3x3 rotmat, vec4 q);
void euler2rotation(vec3 euler,mat3x3 rotmat);

void INS_Att_Resolve(uint8_t AttCalMode, uint8_t SampleNum, vec3 GyroData, vec10 time,
                          vec4 Q, mat3x3 Cbn, vec3 AttData);

void INS_Att_Update(uint8_t AttCalMode, uint8_t SampleNum, vec3 GyroData, vec10 time,
                        vec4 Q, mat3x3 Cbn, vec3 AttData);
void INS_Vel_Resolve(mat3x3 Cbn, vec3 AccData, vec3 VelData, vec10 time);
void INS_Pos_Resolve(vec3 VelData,vec3 PosData,  vec10 time);
void INS_Storage_Sensor(void);
float32_t INS_Cal_Time(void);

/*Alignment functions*/
void INS_InitAlignment(uint8_t mode);
void INS_Align_Acc(float32_t *AccData, float32_t *AttData);
void INS_Align_AccMag(float32_t *AccData, float32_t *MagData,float32_t *AttData);
void INS_Align_Trans(float32_t *AlignData);
void INS_Align_Complex(float32_t *AlignData);
void INS_Align_Motion(float32_t *AttData);
void INS_Alignment(void);

/*define the INS variable value*/
Inertial_Sensor_Info INS_Sensor_Par[DEV_TYPE_NUM] = { { 0 }, { 0 },{0} };   //
Nav_Info INS_Nav_Par = { 0,{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, &INSSolve_Init, &INS_Alignment, &INS_Load_Data, &INS_Storage_Sensor,
                         &INS_Att_Resolve,&INS_Vel_Resolve,&INS_Pos_Resolve,&INS_Cal, &INS_Cal_Time,
                         &quat2rotation, &rotation2euler, &rotation2quat,&euler2rotation};
INS_Cal_Info INS_Cal_Par = { {0},5 };

static float32_t dt=0,lastUpdata=0;  //the time data

typedef struct{
  float32_t Wnbb[MAX_SAMPLE_NUM][AXIS_NUM];
  float32_t WnbbX[MAX_SAMPLE_NUM][AXIS_NUM];
  vec3 WnbbA;
  uint16_t SolveCount;
  float32_t FirstRunFlag;
  float32_t AttResolveFlag;
}Att_Resolve_Info;

static Att_Resolve_Info Att_Resolve_Par={{0},{0},{0},1,0,0};

#define PAR1 1.812101910f
#define PAR2 0.883604505f
#define PAR3 0.538951494f
#define PAR4 0.035051067f
#define PAR5 -0.0006071294342f

#define PAR2_1 0.630158730f
#define PAR2_2 0.015873016f
#define PAR2_3 0.001587302f

void INS_Alignment(void){
	INS_InitAlignment(Dev_Set_Info.Align_Mode);
}
/***************************************************************** 
*  @brief:      initial alignment based on the acc and magnism
*  @Function: 
*  @inparam:
*  @outparam:  
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void INS_InitAlignment(uint8_t mode){
  float32_t timep=0;
  
  switch(mode){   //only use Acceleration to calculate the attitude
    case (AlignAcc):{
      INS_Align_Acc(INS_Sensor_Par[Acc].data, INS_Nav_Par.Att);
      euler2rotation(INS_Nav_Par.Att, INS_Nav_Par.Cbn);
      rotation2quat(INS_Nav_Par.Cbn,INS_Nav_Par.Q);
      INS_Nav_Par.Pos[AxisX]=INS_Nav_Par.Pos[AxisY]=INS_Nav_Par.Pos[AxisZ]=0;
      INS_Nav_Par.Vel[AxisX]=INS_Nav_Par.Vel[AxisY]=INS_Nav_Par.Vel[AxisZ]=0;
      break;
    }
    case(AlignAccMag):{//use acceleration and magnisim to calculate the attitude
      INS_Align_AccMag(INS_Sensor_Par[Acc].data, INS_Sensor_Par[Mag].data, INS_Nav_Par.Att);
      euler2rotation(INS_Nav_Par.Att, INS_Nav_Par.Cbn);
      rotation2quat(INS_Nav_Par.Cbn,INS_Nav_Par.Q);
      INS_Nav_Par.Pos[AxisX]=INS_Nav_Par.Pos[AxisY]=INS_Nav_Par.Pos[AxisZ]=0;
      INS_Nav_Par.Vel[AxisX]=INS_Nav_Par.Vel[AxisY]=INS_Nav_Par.Vel[AxisZ]=0;
      break;
    }
    case(AlignTrans):{
      INS_Align_Trans(INS_Nav_Par.AlignData);
      break;
    }
    case(AlignComplex):{
      INS_Align_Complex(INS_Nav_Par.AlignData);
      break;
    }
    case(AlignMotion):{
      
      break;
    }
    default:break;
  }
  timep=INS_Cal_Time();
}
/***************************************************************** 
*  @brief:          
*  @Function: 
*  @inparam:
*  @outparam:  
*  @Author:       
*  @Date:        170620
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void INS_Align_Acc(float32_t *AccData, float32_t *AttData){
  float32_t g1 = 0;

  g1=Mat_Cal_Info.Norm(AccData,3,1);

#if defined(SYS_MATH_LIB)
    AttData[Roll] = atan2(-AccData[AccY],-AccData[AccZ]);
    AttData[Pitch]= asin(AccData[AccX]/g1);
    AttData[Yaw]  = 0;
#elif defined(ARM_MATH_LIB)
    AttData[Roll] = atan2(-AccData[AccY],-AccData[AccZ]);
    AttData[Pitch]= asin(AccData[AccX]/g1);
    AttData[Yaw]  = 0;

#endif

}
/***************************************************************** 
*  @brief:          
*  @Function: 
*  @inparam:
*  @outparam:  
*  @Author:       
*  @Date:        170620
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void INS_Align_AccMag(float32_t *AccData, float32_t *MagData,float32_t *AttData){
  float32_t g1 = 0,temp1=0,temp2=0;

  g1=Mat_Cal_Info.Norm(AccData,3,1);

}
void INS_Align_Trans(float32_t *AlignData){
//  if()
}
void INS_Align_Complex(float32_t *AlignData){
  
}
void INS_Align_Motion(float32_t *AttData){
}

float32_t INS_Cal_Time(void){
  float32_t now_time=0;

    /*caculate this time and the refresh time*/
#if defined(VS_SIM)             //use vs simulation
    dt = micros();
#elif defined(STM_RUN)
    now_time=micros();
    if(now_time < lastUpdata){
      dt = ((float32_t)(now_time +(0xffff - lastUpdata))/50000000.0f);
    }else{
      dt = ((float32_t)(now_time-lastUpdata)/50000000.0f);
    }
    dt=dt*Dev_Set_Info.Integral_C;
    lastUpdata = now_time;
#endif

  return 0.005;
}
/***************************************************************** 
*  @brief:      INS update the data
*  @File:          
*  @Project:     
*  @Author:     LiuNing
*  @Date:       20170620
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void INS_Att_Resolve(uint8_t AttCalMode, uint8_t SampleNum, vec3 GyroData, vec10 time,
	vec4 Q, mat3x3 Cbn, vec3 AttData){

  if(AttCalMode == Bika){   //bika algorithm to calculate the attitude
    time[0]=INS_Cal_Time();
    INS_Att_Update(AttCalMode, SampleNum, GyroData,time,Q,Cbn,AttData);
  } else if (AttCalMode == RotationVector){

	  time[Att_Resolve_Par.SolveCount-1] = INS_Cal_Time();
    Att_Resolve_Par.Wnbb[Att_Resolve_Par.SolveCount][GyroX] = GyroData[GyroX];
    Att_Resolve_Par.Wnbb[Att_Resolve_Par.SolveCount][GyroY] = GyroData[GyroY];
    Att_Resolve_Par.Wnbb[Att_Resolve_Par.SolveCount][GyroZ] = GyroData[GyroZ];

    Att_Resolve_Par.SolveCount++;
    
    if(Att_Resolve_Par.FirstRunFlag == true){    //wether the rotation vector is the first run
      if(Att_Resolve_Par.SolveCount == SampleNum){
        Att_Resolve_Par.SolveCount      =  1;
        Att_Resolve_Par.AttResolveFlag  =  true;
        Att_Resolve_Par.FirstRunFlag    =  false;
      }
    }else{
      if(Att_Resolve_Par.SolveCount == SampleNum){
        Att_Resolve_Par.SolveCount     = 1;
        Att_Resolve_Par.AttResolveFlag = true;
      }
    }
    
    if(Att_Resolve_Par.AttResolveFlag == true){
      INS_Att_Update(AttCalMode, SampleNum, GyroData,time,Q,Cbn,AttData);
      Att_Resolve_Par.AttResolveFlag = false;
    }
    
  }
}

void INS_Storage_Sensor(void){
  uint16_t i=0,row=0,col=0;
  float32_t Sum[DEV_TYPE_NUM][AXIS_NUM] = { 0 };

  /*storage imu data*/
  for(i=0;i<INS_Cal_Par.BufferSize-1;i++){
    for(row=0;row<DEV_TYPE_NUM;row++){
      for(col=0;col<AXIS_NUM;col++){
		  INS_Sensor_Par[row].BufferData[col][(uint16_t)(INS_Cal_Par.BufferSize - 1 - i)] 
			  = INS_Sensor_Par[row].BufferData[col][(uint16_t)(INS_Cal_Par.BufferSize - 1 - i)-1];
		  Sum[row][col] = Sum[row][col] + INS_Sensor_Par[row].BufferData[col][(uint16_t)(INS_Cal_Par.BufferSize - 1 - i)];
      }
    }
  }
  for(row=0;row<DEV_TYPE_NUM;row++){
    for(col=0;col<AXIS_NUM;col++){
      INS_Sensor_Par[row].BufferData[col][0]=INS_Sensor_Par[row].data[col];
	  Sum[row][col] = Sum[row][col] + INS_Sensor_Par[row].BufferData[col][0];
	  INS_Sensor_Par[row].Mean[col] = Sum[row][col] / INS_Cal_Par.BufferSize;
    }
  }

}

/***************************************************************** 
*  @brief:       Attitude update
*  @Function:    INS_Att_Update
*  @inparam:     
*  @outparam:  
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void INS_Att_Update(uint8_t AttCalMode, uint8_t SampleNum, vec3 GyroData, vec10 time,
                        vec4 Q, mat3x3 Cbn, vec3 AttData){
  uint16_t i=0;
  vec3 Angular_Rate_dt={0};
  float32_t v=0,cos_v=0,sin_v=0,dth=0;
  vec4 quat_tmp;

  if((GyroData[GyroX] !=0) | (GyroData[GyroY] !=0) | (GyroData[GyroZ] !=0)){

     if(AttCalMode == Bika){
        for(i=0;i<AXIS_NUM;i++){
          /*calculate the angular add*/
          Angular_Rate_dt[i]=GyroData[i]*time[0];
        }
     }else if (AttCalMode == RotationVector){

    }
  
  v = sqrt_hf(Mat_Cal_Info.Norm(Angular_Rate_dt,3,1));
	 
#if defined(SYS_MATH_LIB)
	cos_v = cos(v / 2);
	sin_v = sin(v / 2)/v;

#elif defined(ARM_MATH_LIB)
	cos_v = arm_cos_f32(v/2);
	sin_v = arm_sin_f32(v / 2) / v;
#endif
	if(v!=0){
  	quat_tmp[0] = cos_v*Q[0] + sin_v*(Angular_Rate_dt[2] * Q[1] - Angular_Rate_dt[1] * Q[2] + Angular_Rate_dt[0] * Q[3]);
  	quat_tmp[1] = cos_v*Q[1] + sin_v*(-Angular_Rate_dt[2] * Q[0] + Angular_Rate_dt[0] * Q[2] + Angular_Rate_dt[1] * Q[3]);
  	quat_tmp[2] = cos_v*Q[2] + sin_v*(Angular_Rate_dt[1] * Q[0] - Angular_Rate_dt[0] * Q[1] + Angular_Rate_dt[2] * Q[3]);
  	quat_tmp[3] = cos_v*Q[3] + sin_v*(-Angular_Rate_dt[0] * Q[0] - Angular_Rate_dt[1] * Q[1] - Angular_Rate_dt[2] * Q[2]);

  	v = sqrt_hf(Mat_Cal_Info.Norm(quat_tmp, 4, 1));

  	for (i = 0; i < 4; i++){
  		Q[i] = quat_tmp[i] / v;
  	}
  }
 }

 /*update the attitude velocity and postion*/

  quat2rotation(Q, Cbn);
  rotation2euler(Cbn, AttData);
    
}
/***************************************************************** 
*  @brief:          
*  @Function: 
*  @inparam:      Cbn  is rotation matrix
                  AccData is the acceleration data this time
                  VelData is the velocity data
                  time is the refresh time
*  @outparam:  
*  @Author:       
*  @Date:         
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
void INS_Vel_Resolve(mat3x3 Cbn, vec3 AccData, vec3 VelData, vec10 time){
  vec3 an_hat={0};

  // Compute acceleration in navigation coordinate frame and subtract the acceleration due to the earth gravity force. 
  an_hat[0]=Cbn[0]*AccData[AccX]+Cbn[1]*AccData[AccY]+Cbn[2]*AccData[AccZ];
  an_hat[1]=Cbn[3]*AccData[AccX]+Cbn[4]*AccData[AccY]+Cbn[5]*AccData[AccZ];
  an_hat[2]=Cbn[6]*AccData[AccX]+Cbn[7]*AccData[AccY]+Cbn[8]*AccData[AccZ]+g;	

  // Integrate the acceleration to get the velocity
  VelData[AxisX]=VelData[AxisX]+an_hat[0]*time[0];
  VelData[AxisY]=VelData[AxisY]+an_hat[1]*time[0];
  VelData[AxisZ]=VelData[AxisZ]+an_hat[2]*time[0];
}


void INS_Pos_Resolve( vec3 VelData,vec3 PosData, vec10 time){

  // Integrate the velocity to get the position
  PosData[AxisX]=PosData[AxisX]+VelData[AxisX]*time[0];
  PosData[AxisY]=PosData[AxisY]+VelData[AxisY]*time[0];
  PosData[AxisZ]=PosData[AxisZ]+VelData[AxisZ]*time[0];

}


void quat2rotation(vec4 q, mat3x3 rotmat){
  vec6 p;

  p[0]=q[0]*q[0];
  p[1]=q[1]*q[1];
  p[2]=q[2]*q[2];
  p[3]=q[3]*q[3];

  p[4]=p[1]+p[2];

  if (fabs(p[0]+p[3]+p[4]) >espp){
    p[5]=2/(p[0]+p[3]+p[4]);
  }else{
    p[5]=0;
  }

  rotmat[0]=1-p[5]*p[4];			//R(1,1)=1-p(6)*p(5);
  rotmat[4]=1-p[5]*(p[0]+p[2]);	//R(2,2)=1-p(6)*(p(1)+p(3));
  rotmat[8]=1-p[5]*(p[0]+p[1]);	//R(3,3)=1-p(6)*(p(1)+p(2));

  p[0]=p[5]*q[0];					//p(1)=p(6)*q(1); 
  p[1]=p[5]*q[1];					//p(2)=p(6)*q(2);
  p[4]=p[5]*q[2]*q[3];			//p(5)=p(6)*q(3)*q(4);
  p[5]=p[0]*q[1];					//p(6)=p(1)*q(2);

  rotmat[1]=p[5]-p[4];			//R(1,2)=p(6)-p(5);
  rotmat[3]=p[5]+p[4];			//R(2,1)=p(6)+p(5);

  p[4]=p[1]*q[3];					//p(5)=p(2)*q(4);
  p[5]=p[0]*q[2];					//p(6)=p(1)*q(3);

  rotmat[2]=p[5]+p[4];			//R(1,3)=p(6)+p(5);
  rotmat[6]=p[5]-p[4];			//R(3,1)=p(6)-p(5);

  p[4]=p[0]*q[3];					//p(5)=p(1)*q(4);
  p[5]=p[1]*q[2];					//p(6)=p(2)*q(3);

  rotmat[5]=p[5]-p[4];			//R(2,3)=p(6)-p(5);
  rotmat[7]=p[5]+p[4];			//R(3,2)=p(6)+p(5);
}

void rotation2euler(mat3x3 rotmat , vec3 euler){
	float64_t i = (float64_t)sqrt_hf(1 - rotmat[6] * rotmat[6]);
	float64_t j = 0;

	if (fabs(i)<0.0001){
		i = 0.001;
	}
	j = (float64_t)rotmat[6] / i;
	if (fabs(j)<0.0001){
		j = 0.000001;
	}
	euler[0] = (float32_t)(atan2((float64_t)rotmat[7], (float64_t)rotmat[8]));	//atan2( R(3,2), R(3,3) );
	euler[1] = (float32_t)(-atan(j));			//asin( -R(3,1) );
	euler[2] = (float32_t)(atan2((float64_t)rotmat[3], (float64_t)rotmat[0]));	//atan2( R(2,1), R(1,1));
}

void rotation2quat(mat3x3 rotmat,vec4 q){
	float32_t S=0,T=0;
	// For checking robustness of DCM, diagonal elements
	T = 1 + rotmat[0] + rotmat[4]+rotmat[8];  // 1+diag(R)
	
	// Calculate quaternion based on largest diagonal element
	if(T > (0.00000001f)){
		S = 0.5f / sqrt_hf(T);
		q[3] = 0.25f / S;
		q[0] =(rotmat[7]-rotmat[5]) * S;	//(R(3,2) - R(2,3))*S
		q[1] =(rotmat[2]-rotmat[6]) * S;  	//( R(1,3) - R(3,1) ) * S;
		q[2] =(rotmat[3]-rotmat[1]) * S;	// ( R(2,1) - R(1,2) ) * S;
		}
	else if( (rotmat[0] > rotmat[4] ) && (rotmat[0] > rotmat[8]) ) //(R(1,1) > R(2,2)) && (R(1,1) > R(3,3)) 
	{
		S = sqrt_hf(1 + rotmat[0]-rotmat[4]-rotmat[8]) * 2; //S=sqrt_hf(1 + R(1,1) - R(2,2) - R(3,3)) * 2
		q[3] =(rotmat[6]-rotmat[5])/S;		//(R(3,1) - R(2,3)) / S;
		q[0] = 0.25f * S;
		q[1] = (rotmat[1]+rotmat[3])/S;		//(R(1,2) + R(2,1)) / S;
		q[2] = (rotmat[2]+rotmat[6])/S;		//(R(1,3) + R(3,1)) / S;
		}
	else if (rotmat[4]>rotmat[8])		//(R(2,2) > R(3,3))
	{
		S = sqrt_hf(1+rotmat[4]-rotmat[0]-rotmat[8])*2;	//sqrt_hf( 1 + R(2,2) - R(1,1) - R(3,3) ) * 2; 
		q[3] = (rotmat[2]-rotmat[6])/S;						//(R(1,3) - R(3,1)) / S;
		q[0] =	(rotmat[1]+rotmat[3])/S;					//(R(1,2) + R(2,1)) / S;
		q[1] = 0.25f * S;
		q[2] = (rotmat[5]+rotmat[7])/S;						//(R(2,3) + R(3,2)) / S;
		}
	else{
		S=sqrt_hf(1+rotmat[8]-rotmat[0]-rotmat[4])*2;		//S = sqrt_hf( 1 + R(3,3) - R(1,1) - R(2,2) ) * 2; 
		q[3] = (rotmat[3]-rotmat[1])/S;						//(R(2,1) - R(1,2)) / S;
		q[0] = (rotmat[2]+rotmat[6])/S;						//(R(1,3) + R(3,1)) / S;
		q[1] = (rotmat[1]+rotmat[3])/S;						//(R(1,2) + R(2,1)) / S;
		q[2] = 0.25f * S;
		}
	
	
	//Normalize the quaternion
	T=sqrt_hf(Mat_Cal_Info.Norm(q,4,1));
	
	q[0]=q[0]/T;
	q[1]=q[1]/T;
	q[2]=q[2]/T;
	q[3]=q[3]/T;

}

void euler2rotation(vec3 euler,mat3x3 rotmat){
  float32_t sin_phi=0,cos_phi=0,sin_theta=0,cos_theta=0,sin_psi=0,cos_psi=0;

#if defined(SYS_MATH_LIB)
  // Trigonometric value variables	
  sin_phi   = (float32_t)sin(euler[0]);
  cos_phi   = (float32_t)cos(euler[0]);
  sin_theta = (float32_t)sin(euler[1]);
  cos_theta = (float32_t)cos(euler[1]);
  sin_psi   = (float32_t)sin(euler[2]);
  cos_psi   = (float32_t)cos(euler[2]);


#elif defined(ARM_MATH_LIB)
  // Trigonometric value variables	
  sin_phi = arm_sin_f32(euler[0]);
  cos_phi = arm_cos_f32(euler[0]);
  sin_theta =arm_sin_f32(euler[1]);
  cos_theta =arm_cos_f32(euler[1]);
  sin_psi = arm_sin_f32(euler[2]);
  cos_psi = arm_cos_f32(euler[2]);

#endif

  rotmat[0]=cos_psi*cos_theta;								//Matrix element [1,1]
  rotmat[3]=sin_psi*cos_theta;								//Matrix element [1,2]
  rotmat[6]=-sin_theta;										//Matrix element [1,3]
  rotmat[1]=(-sin_psi*cos_phi) + cos_psi*(sin_theta*sin_phi);		//Matrix element [2,1]
  rotmat[4]=(cos_psi*cos_phi) + sin_psi*(sin_theta*sin_phi);		//Matrix element [2,2]
  rotmat[7]=cos_theta*sin_phi;								//Matrix element [2,3] 
  rotmat[2]=(sin_psi*sin_phi) + cos_psi*(sin_theta*cos_phi);		//Matrix element [3,1]
  rotmat[5]=(-cos_psi*sin_phi) + sin_psi*(sin_theta*cos_phi);		//Matrix element [3,2]		          
  rotmat[8]=cos_theta*cos_phi;								//Matrix element [3,3]	
}

float32_t sqrt_hf(float32_t arg){
  if (arg<0){
	  arg = 0;
  }
  return (float32_t)sqrt(arg);
}


