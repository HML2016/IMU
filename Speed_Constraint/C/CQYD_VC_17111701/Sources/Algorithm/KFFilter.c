/**/
#include "KFFilter.h"

/*function declaration*/
matDSxDP tempDSxDP={0};
matDSxDS tempDSxDS1 = { 0 }, tempDSxDS2 = { 0 }, tempDSxDS3 = { 0 },tempDSxDS4 = { 0 };
matDPxDS tempDPxDS = { 0 };
matDOxDO tempDOxDO = { 0 }, tempDOxDO2 = { 0 }, tempDOxDO3= { 0 };
matDSxDO tempDSxDO1 = { 0 }, tempDSxDO2 = { 0 };


void KFFilter_Time_Updata(void){

  // Update the filter state covariance matrix P.    P=F*P*F'+G*Q*G';
  Mat_Cal_Info.Inver(Kalman_Par.F, DIM_STATE, DIM_STATE, tempDSxDS1);
  Mat_Cal_Info.MulM(Kalman_Par.P, tempDSxDS1, DIM_STATE, DIM_STATE, DIM_STATE, tempDSxDS2);
  Mat_Cal_Info.MulM(Kalman_Par.F, tempDSxDS2, DIM_STATE, DIM_STATE, DIM_STATE, tempDSxDS3);

  //G*Q*G'
  Mat_Cal_Info.MulM(Kalman_Par.G, Kalman_Par.Q, DIM_STATE, DIM_Proc, DIM_Proc, tempDSxDP);
  Mat_Cal_Info.Inver(Kalman_Par.G, DIM_STATE, DIM_Proc, tempDPxDS);
  Mat_Cal_Info.MulM(tempDSxDP, tempDPxDS, DIM_STATE, DIM_Proc, DIM_STATE, tempDSxDS1);
  
  //P=F*P*F'+G*Q*G'
  Mat_Cal_Info.Add(tempDSxDS1,tempDSxDS2,DIM_STATE,DIM_STATE,Kalman_Par.P);

  //Make sure the filter state covariance matrix is symmetric   P=(P+P')/2;
  Mat_Cal_Info.Inver(Kalman_Par.P,DIM_STATE,DIM_STATE,tempDSxDS2);
  Mat_Cal_Info.Add(Kalman_Par.P, tempDSxDS2, DIM_STATE, DIM_STATE, tempDSxDS1);
  Mat_Cal_Info.MulD(tempDSxDS1,DIM_STATE,DIM_STATE,0.5f,Kalman_Par.P);

}

void KFFilter_Measure_Update(void){
  //Calculate the Kalman filter gain     K=(P*H')/(H*P*H'+R);

	/*H*P*H'*/
	//Mat_Cal_Info.MulInver(Kalman_Par.P, Kalman_Par.H, DIM_STATE, DIM_STATE, DIM_OBS, tempDSxDO1);  //P*H'/
	Mat_Cal_Info.Inver(Kalman_Par.H, DIM_OBS, DIM_STATE, tempDSxDO2);
	Mat_Cal_Info.MulM(Kalman_Par.P, tempDSxDO2, DIM_STATE, DIM_STATE, DIM_OBS, tempDSxDO1);
	Mat_Cal_Info.MulM(Kalman_Par.H, tempDSxDO1, DIM_OBS,DIM_STATE, DIM_OBS, tempDOxDO);
	Mat_Cal_Info.Add(tempDOxDO, Kalman_Par.R, DIM_OBS, DIM_OBS, tempDOxDO2);
	/*/(H*P*H'+R) E-1*/
	Mat_Cal_Info.Opp(tempDOxDO2, DIM_OBS, tempDOxDO3);
	Mat_Cal_Info.MulM(tempDSxDO1, tempDOxDO3, DIM_STATE, DIM_OBS, DIM_OBS, Kalman_Par.K);

	//Update the filter state covariance matrix P.		P = (Id - K*H)*P;
	Mat_Cal_Info.MulM(Kalman_Par.K, Kalman_Par.H, DIM_STATE, DIM_OBS, DIM_STATE, tempDSxDS4); //K*H
	Mat_Cal_Info.Sub(Kalman_Par.Id, tempDSxDS4, DIM_STATE, DIM_STATE, tempDSxDS1);  //Id - K*H
	Mat_Cal_Info.MulM(tempDSxDS1, Kalman_Par.P, DIM_STATE, DIM_STATE, DIM_STATE, tempDSxDS4); //P=(Id - K*H)*P
	
	//% Make sure the filter state covariance matrix is symmetric.		P = (P + P')/2;
	Mat_Cal_Info.Inver(tempDSxDS4, DIM_STATE, DIM_STATE, tempDSxDS1);    //P'
	Mat_Cal_Info.Add(tempDSxDS4, tempDSxDS1, DIM_STATE, DIM_STATE, tempDSxDS2);
	Mat_Cal_Info.MulD(tempDSxDS2, DIM_STATE, DIM_STATE, 0.5f, Kalman_Par.P);

}

