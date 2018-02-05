/***************************************************************** 
*  @brief:         
*  @File:          
*  @Project:     
*  @Author:      
*  @Date:       170601
*  @CopyRight:
*  @Version:     
*  @Description:
*****************************************************************/
#include "KFFilter.h"

/*variable define*/
static mat3x3 Id = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
matDSxDS Fc = { 0 }, tempDSxDS = {0};
matDSxDP Gc = { 0 };
vec3 ft = { 0 };

mat3x3 dcmError = { 0 };
mat3x3 M3temp1 = { 0 }, M3temp2 = { 0 };

Kalman_Cal_Info Kalman_Cal_Func = {&KFFilter_Init,&KFFilter_Cal_State,&KFFilter_State_Update,&KFFilter_Time_Updata,&KFFilter_Measure_Update};
Kalman_Set_Info Kalman_Set_Par = { { 0.5f }, { 0.5f*ATR }, { 1E-7f }, { ATR*1E-7f }, { 0.01f }, { 1E-5 }, { 1E-5f }, { 0.1f*ATR }, { 0.3f }, 
                                   { 0.3f*ATR }, { 1E-4f }, { 1E-5f }, 0, 0,0};
Kalman_Info Kalman_Par = { { 0 }, { 1 }, { 1 }, { 1 }, { 1 }, { 1 }, { 1 }, { 1 }, { 1 }, { 0 }, { 0 } };

void KFFilter_Init(void){

//	Kalman_Set_Par.BuffData = Command_Par_Info[COMMAND_KALMAN].Buffer;
	Kalman_Set_Par.SigmaAcc[0] = 0.08;
	Kalman_Set_Par.SigmaAcc[1] = 0.08;
	Kalman_Set_Par.SigmaAcc[2] = 0.08;

	Kalman_Set_Par.SigmaGyro[0] = 0.08 * 3.14 / 180;
	Kalman_Set_Par.SigmaGyro[1] = 0.08 * 3.14 / 180;
	Kalman_Set_Par.SigmaGyro[2] = 0.08 * 3.14 / 180;

	Kalman_Set_Par.AccDrNoise[0] = 1e-7;
	Kalman_Set_Par.AccDrNoise[1] = 1e-7;
	Kalman_Set_Par.AccDrNoise[2] = 1e-7;

	Kalman_Set_Par.GyroDrNoise[0] = 0.5f*ATR;
	Kalman_Set_Par.GyroDrNoise[1] = 0.5f*ATR;
	Kalman_Set_Par.GyroDrNoise[2] = 0.5f*ATR;

	Kalman_Set_Par.SigmaVel[0] = 0.0001;
	Kalman_Set_Par.SigmaVel[1] = 0.0001;
	Kalman_Set_Par.SigmaVel[2] = 0.0001;

	Kalman_Set_Par.SigmaInPos[0] = 1E-5;
	Kalman_Set_Par.SigmaInPos[1] = 1E-5;
	Kalman_Set_Par.SigmaInPos[2] = 1E-5;

	Kalman_Set_Par.SigmaInVel[0] = 1E-5;
	Kalman_Set_Par.SigmaInVel[1] = 1E-5;
	Kalman_Set_Par.SigmaInVel[2] = 1E-5;

	Kalman_Set_Par.SigmaInAtt[0] = 0.1f*ATR;
	Kalman_Set_Par.SigmaInAtt[1] = 0.1f*ATR;
	Kalman_Set_Par.SigmaInAtt[2] = 0.1f*ATR; 
  
	Mat_Cal_Info.EG(Kalman_Par.Id, DIM_STATE);

	//H
	Kalman_Par.H[0] = 0; Kalman_Par.H[1] = 0; Kalman_Par.H[2] = 0; Kalman_Par.H[3] = 1; Kalman_Par.H[4] = 0; Kalman_Par.H[5] = 0;
	Kalman_Par.H[6] = 0; Kalman_Par.H[7] = 0; Kalman_Par.H[8] = 0; Kalman_Par.H[9] = 0; Kalman_Par.H[10] = 0; Kalman_Par.H[11] = 0;
	Kalman_Par.H[12] = 0; Kalman_Par.H[13] = 1; Kalman_Par.H[14] = 0; Kalman_Par.H[15] = 0; Kalman_Par.H[16] = 0; Kalman_Par.H[17] =0;
	Kalman_Par.H[18] = 0; Kalman_Par.H[19] = 0; Kalman_Par.H[20] = 0; Kalman_Par.H[21] = 0; Kalman_Par.H[22] = 0; Kalman_Par.H[23] = 1;
	Kalman_Par.H[24] = 0; Kalman_Par.H[25] = 0; Kalman_Par.H[26] = 0;  
	//P the first row
	Mat_Cal_Info.EG(Kalman_Par.P, DIM_STATE);
	//initial the position
	Kalman_Par.P[0] = Kalman_Set_Par.SigmaInPos[PosN] * Kalman_Set_Par.SigmaInPos[PosN];
	Kalman_Par.P[10] = Kalman_Set_Par.SigmaInPos[PosE] * Kalman_Set_Par.SigmaInPos[PosE];
	Kalman_Par.P[20] = Kalman_Set_Par.SigmaInPos[PosD] * Kalman_Set_Par.SigmaInPos[PosD];
	//initial the velocity
	Kalman_Par.P[30] = Kalman_Set_Par.SigmaInVel[VelN] * Kalman_Set_Par.SigmaInVel[VelN];
	Kalman_Par.P[40] = Kalman_Set_Par.SigmaInVel[VelE] * Kalman_Set_Par.SigmaInVel[VelE];
	Kalman_Par.P[50] = Kalman_Set_Par.SigmaInVel[VelD] * Kalman_Set_Par.SigmaInVel[VelD];
	//initial the velocity
	Kalman_Par.P[60] = Kalman_Set_Par.SigmaInAtt[Roll] * Kalman_Set_Par.SigmaInAtt[Roll];
	Kalman_Par.P[70] = Kalman_Set_Par.SigmaInAtt[Pitch] * Kalman_Set_Par.SigmaInAtt[Pitch];
	Kalman_Par.P[80] = Kalman_Set_Par.SigmaInAtt[Yaw] * Kalman_Set_Par.SigmaInAtt[Yaw];

	//Q
	Mat_Cal_Info.EG(Kalman_Par.Q, DIM_Proc);
	Kalman_Par.Q[0] = Kalman_Set_Par.SigmaAcc[AxisX] * Kalman_Set_Par.SigmaAcc[AxisX];
	Kalman_Par.Q[7] = Kalman_Set_Par.SigmaAcc[AxisY] * Kalman_Set_Par.SigmaAcc[AxisY];
	Kalman_Par.Q[14] = Kalman_Set_Par.SigmaAcc[AxisZ] * Kalman_Set_Par.SigmaAcc[AxisZ];

	Kalman_Par.Q[21] = Kalman_Set_Par.SigmaGyro[AxisX] * Kalman_Set_Par.SigmaGyro[AxisX];
	Kalman_Par.Q[28] = Kalman_Set_Par.SigmaGyro[AxisY] * Kalman_Set_Par.SigmaGyro[AxisY];
	Kalman_Par.Q[35] = Kalman_Set_Par.SigmaGyro[AxisZ] * Kalman_Set_Par.SigmaGyro[AxisZ];
	//R
	Mat_Cal_Info.EG(Kalman_Par.R, DIM_OBS);
	Kalman_Par.R[0] = Kalman_Set_Par.SigmaVel[AxisX] * Kalman_Set_Par.SigmaVel[AxisX];
	Kalman_Par.R[4] = Kalman_Set_Par.SigmaVel[AxisY] * Kalman_Set_Par.SigmaVel[AxisY];
	Kalman_Par.R[8] = Kalman_Set_Par.SigmaVel[AxisZ] * Kalman_Set_Par.SigmaVel[AxisZ];

}

void KFFilter_Cal_State(vec3 GyroData,vec3 AccData){
	//INS_Nav_Par.ReadTime();
//#if (DIM_STATE == 9)

	float32_t dt = INS_Nav_Par.ReadTime();
	Mat_Cal_Info.MulM(INS_Nav_Par.Cbn, AccData, 3, 3, 1, ft);
	//the first row
	Fc[0 ] = 0; Fc[1 ] = 0; Fc[2 ] = 0; Fc[3 ] = 1; Fc[4 ] = 0; Fc[5 ] = 0; Fc[6 ] = 0; Fc[7 ] = 0; Fc[8 ] = 0;
	Fc[9 ] = 0; Fc[10] = 0; Fc[11] = 0; Fc[12] = 0; Fc[13] = 1; Fc[14] = 0; Fc[15] = 0; Fc[16] = 0; Fc[17] = 0;
	Fc[18] = 0; Fc[19] = 0; Fc[20] = 0; Fc[21] = 0; Fc[22] = 0; Fc[23] = 1; Fc[24] = 0; Fc[25] = 0; Fc[26] = 0;

	Fc[27] = 0; Fc[28] = 0; Fc[29] = 0; Fc[30] = 0; Fc[31] = 0; Fc[32] = 0; Fc[33] = 0;      Fc[34] = -ft[2]; Fc[35] = ft[1];
	Fc[36] = 0; Fc[37] = 0; Fc[38] = 0; Fc[39] = 0; Fc[40] = 0; Fc[41] = 0; Fc[42] = ft[2];  Fc[43] = 0;      Fc[44] = -ft[0];
	Fc[45] = 0; Fc[46] = 0; Fc[47] = 0; Fc[48] = 0; Fc[49] = 0; Fc[50] = 0; Fc[51] = -ft[1]; Fc[52] = ft[0];  Fc[53] = 0;

	Fc[54] = 0; Fc[55] = 0; Fc[56] = 0; Fc[57] = 0; Fc[58] = 0; Fc[59] = 0; Fc[60] = 0; Fc[61] = 0; Fc[62] = 0;
	Fc[63] = 0; Fc[64] = 0; Fc[65] = 0; Fc[66] = 0; Fc[67] = 0; Fc[68] = 0; Fc[69] = 0; Fc[70] = 0; Fc[71] = 0;
	Fc[72] = 0; Fc[73] = 0; Fc[74] = 0; Fc[75] = 0; Fc[76] = 0; Fc[77] = 0; Fc[78] = 0; Fc[79] = 0; Fc[80] = 0;

	Gc[0] = 0; Gc[1] = 0; Gc[2] = 0; Gc[3] = 0; Gc[4] = 0; Gc[5] = 0;
	Gc[6] = 0; Gc[7] = 0; Gc[8] = 0; Gc[9] = 0; Gc[10] = 0; Gc[11] = 0;
	Gc[12] = 0; Gc[13] = 0; Gc[14] = 0; Gc[15] = 0; Gc[16] = 0; Gc[17] = 0;

	Gc[18] = INS_Nav_Par.Cbn[0]; Gc[19] = INS_Nav_Par.Cbn[1]; Gc[20] = INS_Nav_Par.Cbn[2]; Gc[21] = 0; Gc[22] = 0; Gc[23] = 0;
	Gc[24] = INS_Nav_Par.Cbn[3]; Gc[25] = INS_Nav_Par.Cbn[4]; Gc[26] = INS_Nav_Par.Cbn[5]; Gc[27] = 0; Gc[28] = 0; Gc[29] = 0;
	Gc[30] = INS_Nav_Par.Cbn[6]; Gc[31] = INS_Nav_Par.Cbn[7]; Gc[32] = INS_Nav_Par.Cbn[8]; Gc[33] = 0; Gc[34] = 0; Gc[35] = 0;

	Gc[36] = 0; Gc[37] = 0; Gc[38] = 0; Gc[39] = -INS_Nav_Par.Cbn[0]; Gc[40] = -INS_Nav_Par.Cbn[1]; Gc[41] = -INS_Nav_Par.Cbn[2];
	Gc[42] = 0; Gc[43] = 0; Gc[44] = 0; Gc[45] = -INS_Nav_Par.Cbn[3]; Gc[46] = -INS_Nav_Par.Cbn[4]; Gc[47] = -INS_Nav_Par.Cbn[5];
	Gc[48] = 0; Gc[49] = 0; Gc[50] = 0; Gc[51] = -INS_Nav_Par.Cbn[6]; Gc[52] = -INS_Nav_Par.Cbn[7]; Gc[53] = -INS_Nav_Par.Cbn[8];
//#endif
	Mat_Cal_Info.MulD(Fc, DIM_STATE, DIM_STATE, INS_Cal_Par.dt[0], tempDSxDS);
	Mat_Cal_Info.Add(tempDSxDS,Kalman_Par.Id, DIM_STATE, DIM_STATE, Kalman_Par.F);
	Mat_Cal_Info.MulD(Gc, DIM_STATE, DIM_Proc, INS_Cal_Par.dt[0], Kalman_Par.G);

}
void KFFilter_State_Update(void){
	uint8_t i = 0;
	/* Calculate the prediction error. Since the detector hypothesis
	 is that the platform has zero velocity, the prediction error is
	 equal to zero minu the estimated velocity.  */
	Kalman_Par.Z[0] = 0 - INS_Nav_Par.Vel[VelN];     //oberserve refresh, add the velocity constraint
	Kalman_Par.Z[1] = 0 - INS_Nav_Par.Vel[VelE];
	Kalman_Par.Z[2] = 0 - INS_Nav_Par.Vel[VelD];

	/*Estimation of the perturbations in the estimated navigation states   dx=K*z*/
	Mat_Cal_Info.MulM(Kalman_Par.K, Kalman_Par.Z, DIM_STATE, DIM_OBS, 1, Kalman_Par.dX);

	/*Correct the navigation state using the estimated perturbations. */
	INS_Nav_Par.Pos[PosN] = INS_Nav_Par.Pos[PosN] + Kalman_Par.dX[0];
	INS_Nav_Par.Pos[PosE] = INS_Nav_Par.Pos[PosE] + Kalman_Par.dX[1];
	INS_Nav_Par.Pos[PosD] = INS_Nav_Par.Pos[PosD] + Kalman_Par.dX[2];

	INS_Nav_Par.Vel[VelN] = INS_Nav_Par.Vel[VelN] + Kalman_Par.dX[3];
	INS_Nav_Par.Vel[VelE] = INS_Nav_Par.Vel[VelE] + Kalman_Par.dX[4];
	INS_Nav_Par.Vel[VelD] = INS_Nav_Par.Vel[VelD] + Kalman_Par.dX[5];

	INS_Nav_Par.Att[Roll] = INS_Nav_Par.Att[Roll] + Kalman_Par.dX[6];
	INS_Nav_Par.Att[Pitch] = INS_Nav_Par.Att[Pitch] + Kalman_Par.dX[7];
	INS_Nav_Par.Att[Yaw] = INS_Nav_Par.Att[Yaw] + Kalman_Par.dX[8];
	
	/*Correct the rotation matrics*/
	dcmError[0] = 0;                 dcmError[1] = -Kalman_Par.dX[8]; dcmError[2] = Kalman_Par.dX[7];
	dcmError[3] =  Kalman_Par.dX[8]; dcmError[4] = 0;                 dcmError[5] = -Kalman_Par.dX[6];
	dcmError[6] = -Kalman_Par.dX[7]; dcmError[7] = Kalman_Par.dX[6];  dcmError[8] = 0;

	/*Convert quaternion to a rotation matrix*/
	Mat_Cal_Info.Sub(Id, dcmError, 3, 3, M3temp1);
	Mat_Cal_Info.MulM(M3temp1, INS_Nav_Par.Cbn, 3, 3, 3, M3temp2);
	for (i = 0; i < 9;i++){
		INS_Nav_Par.Cbn[i] = M3temp2[i];
	}
	INS_Nav_Par.dcm2e(INS_Nav_Par.Cbn, INS_Nav_Par.Att);
	INS_Nav_Par.dcm2q(INS_Nav_Par.Cbn, INS_Nav_Par.Q);

}

