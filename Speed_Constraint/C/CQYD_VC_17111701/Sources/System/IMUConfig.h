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
#ifndef __IMUCONFIG_H__
#define __IMUCONFIG_H__
/*------------------------IMU parameters----------------------------*/
//error compentation sequence
#define DEV_GX  0
#define DEV_GY  1
#define DEV_GZ  2

#define DEV_AX  3
#define DEV_AY  4
#define DEV_AZ  5

#define DEV_MX  6
#define DEV_MY  7
#define DEV_MZ  8

/*INS calculate */
#define INS_DEV_NUM 9   //set the calibration device number

#define DEV_TYPE_NUM 3
#define AXIS_NUM 3     //mearsurment axis number
#define GYRO_NUM 3     //gyro number
#define ACC_NUM  3     //acc number
#define MAG_NUM  3     //mag number
#define BARO_NUM 1     //baro number
/*INS information set*/
#define BASIC_STATE_DIM          3    //the basic state dimentions


#if defined(VS_SIM)

#define DEV_NUM 14
#define TIME 0
#define GX 1
#define GY 2
#define GZ 3
#define AX 4
#define AY 5
#define AZ 6
#define MX 7
#define MY 8
#define MZ 9
#define BARO 10
#else
#define DEV_NUM 9
#endif

#define IMU_AX 1
#define IMU_AY 0
#define IMU_AZ 2
#define IMU_TEMP 3
#define IMU_GX 5
#define IMU_GY 4
#define IMU_GZ 6
#define IMU_MX 7
#define IMU_MY 8
#define IMU_MZ 9

#define IMU_U2 0
#define IMU_U3 1
#define IMU_U4 2
#define IMU_U5 3
#define IMU_U6 4
#define IMU_U7 5
#define IMU_U8 6
#define IMU_U9 7

enum Sensor_Type{
	Gyro = 0,
	Acc = 1,
	Mag = 2,
	Baro = 3
};

enum Axis_Type{
	AxisX = 0,
	AxisY = 1,
	AxisZ = 2
};

enum Gyro_Type{
	GyroX = 0,
	GyroY = 1,
	GyroZ = 2
};

enum Acc_Type{
	AccX = 0,
	AccY = 1,
	AccZ = 2
};

enum Mag_Type{
	MagX = 0,
	MagY = 1,
	MagZ = 2
};

enum Att_Type{
	Roll = 0,
	Pitch,
	Yaw
};

enum Vel_Type{
	VelN = 0,
	VelE,
	VelD
};

enum Pos_Type{
	PosN = 0,
	PosE,
	PosD
};

#if defined(VS_SIM)

#elif defined(STM_RUN)

#endif

#endif
