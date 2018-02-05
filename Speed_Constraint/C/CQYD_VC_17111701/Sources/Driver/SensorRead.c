/***************************************************************** 
*  @brief:       Sensor Input Through External ADC/ Internal ADC/FMC  
*  @File:        SensorRead.c
*  @Project:     MIMU
*  @Author:      Liu Ning
*  @Date:        20170611 
*  @CopyRight:   Copyright (c) 2017 HDNT, ISC License (open source)
*  @Version:     V1.0
*  @Description: SensorRead file
*****************************************************************/
#include "SensorRead.h"
//#include "BaroRead.h"

#define IMU_SDI_H   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)    
#define IMU_SDI_L   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)

#define IMU_SCK_H   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET)
#define IMU_SCK_L   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET)

#define IMU_CSM_H   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET)
#define IMU_CSM_L   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET)


#define SDO0 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)
#define SDO1 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)
#define SDO2 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)
#define SDO3 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)
#define SDO4 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)
#define SDO5 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)
#define SDO6 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)
#define SDO7 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)


//9250ÉèÖÃ
#define Diff_NUM 2
#define IMU_DEV_NUM 11
#define READ_9250_NUM 22
#define READ_8975_NUM 6
#define IMU_NUM      8       //IMU¸öÊý
#define MPU6555_InitRegNum 15
#define AK8963_InitRegNum  2

uint32_t lastUpdate, now;

#define Byte16(Type, ByteH, ByteL)  ((Type)((((uint16_t)(ByteH))<<8) | ((uint16_t)(ByteL))))

uint16_t IMU_Data[IMU_NUM][IMU_DEV_NUM]={0};
float32_t IMU_Pre_Data[IMU_NUM][IMU_DEV_NUM]={0};  //Ô¤´¦ÀíÊý¾Ý
float32_t IMU_Sol_Data[Diff_NUM][IMU_DEV_NUM]={0};

  uint8_t temp;
	uint8_t magg[2]={0};

float32_t Calibration_Coe[IMU_DEV_NUM]=
{       MPU9255A_8g, MPU9255A_8g, (MPU9255A_8g),
	      MPU9255T_85degC,      
	      MPU9255G_500dps, MPU9255G_500dps, (MPU9255G_500dps),
	      MPU9255M_4800uT, MPU9255M_4800uT, MPU9255M_4800uT,1,
	      
};


void delay(int i)
{
	for(;i>0;i--);
}

uint8_t Sensor_Check(void)
{
  while(MPU9255_Config_Check() != SUCCESS);
	return SUCCESS;
}

void Sensor_Init(void)
{
//  MS561101BA_init();//Ñ¹Ç¿ ³õÊ¼»¯
	delay_ms(100);
	MPU9255_Config_Check();
//	Initial_System_Timer();//³õÊ¼»¯TIM5 32Î»¶¨Ê±Æ÷£¬ÓÃÓÚ×öÏµÍ³µÄÊ±ÖÓ¡
	
	now = micros();  //
  lastUpdate = now;	//
}

void Sensor_Updata(void)
{
//		MS5611BA_Routing();
    Sensor_Read();
	  //now = micros();
}


void SPI_RW(u8 WriteByte, u8 *Readbuf)
{
	uint8_t i=0,j;
	//uint8_t temp;
	
	IMU_SCK_H;
	delay(100);
	for(i=0;i<8;i++)
	{
		IMU_SCK_L; //SCLK ?1
		delay(100);
		if(((WriteByte >> (7-i)) & 0x01)==1)
		{
			IMU_SDI_H;          //?????
		}
		else
		{
			IMU_SDI_L;
		}
		
		for(j=0;j<IMU_NUM;j++)
		{
			Readbuf[j]=Readbuf[j]<<1;
		}
		
   temp = (uint8_t)((GPIOB->IDR)>>8);  //??????
		
		for(j=0;j<IMU_NUM;j++)
		{
			Readbuf[j]=Readbuf[j] | ((temp >> (j)) & 0x01);   //??????
		}
		
		IMU_SCK_H;
		delay(10);
	}
	  IMU_SCK_H;
	delay(20);
}

/////2017.07.27

void MPU9255_ReadReg( uint8_t ReadAddr, uint8_t *ReadData )
{
	uint8_t databuf[IMU_NUM];
  IMU_CSM_L;
  SPI_RW(0x80 | ReadAddr,databuf);
  SPI_RW(0xFF,ReadData);
  IMU_CSM_H;
}

void MPU9255_WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
  uint8_t databuf[IMU_NUM];
	IMU_CSM_L;
  SPI_RW(WriteAddr,databuf);
  SPI_RW(WriteData,databuf);
  IMU_CSM_H;
	delay_us(10);///////////////////////////////////////////////////////////////////////
}

void MPU9255_ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes )
{
  uint8_t i = 0,j=0;
  uint8_t databuf[IMU_NUM];
  IMU_CSM_L;
  SPI_RW(0x80 | ReadAddr,databuf);
  for(i=0; i<Bytes; i++)
  {
		SPI_RW(0xFF,databuf);
		for(j=0;j<IMU_NUM;j++)
		{
			ReadBuf[i+Bytes*j]=databuf[j];
		}
	}	
  IMU_CSM_H;
}

void MPU9255_Mag_WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
  uint8_t timeout = 0;
  uint8_t status[IMU_NUM] = {0};

  MPU9255_WriteReg(MPU6555_I2C_SLV4_REG, WriteAddr);
  MPU9255_WriteReg(MPU6555_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
  MPU9255_WriteReg(MPU6555_I2C_SLV4_DO, WriteData);
  MPU9255_WriteReg(MPU6555_I2C_SLV4_CTRL, 0x80);

  do {
     MPU9255_ReadReg(MPU6555_I2C_MST_STATUS,status);
  } while(((status[0] & 0x40) == 0) && (timeout++ < 50));
	delay_ms(1);
}

void MPU9255_Mag_ReadReg( uint8_t ReadAddr,uint8_t* ReadBuf )
{
	
	MPU9255_WriteReg(MPU6555_I2C_SLV0_REG, ReadAddr);
	MPU9255_WriteReg(MPU6555_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
	MPU9255_WriteReg(MPU6555_I2C_SLV0_CTRL, 0x81);

	delay_ms(2);
	MPU9255_ReadReg(MPU6555_EXT_SENS_DATA_00,ReadBuf);
}


void MPU9255_Config( void )
{
	
	IMU_CSM_H;
}

uint8_t MPU9255_Check(void)
{
  uint8_t DeviceID[IMU_NUM] = {0x00};
	uint8_t DeviceID2[IMU_NUM] = {0x00};
	uint8_t i;

  /* MPU6500 */
//  DeviceID = 0x00;
  MPU9255_ReadReg(MPU6555_WHO_AM_I, DeviceID);
	for(i=0;i<IMU_NUM;i++)
	{
	  if(DeviceID[i] != MPU6555_Device_ID)
    return ERROR;
	}

  MPU9255_Mag_ReadReg(AK8963_WIA, DeviceID2);   // Read Data
	for(i=0;i<IMU_NUM;i++)
	{
    if(DeviceID2[i] != AK8963_Device_ID)
    return ERROR;
	}

  return SUCCESS;
}


uint8_t MPU9255_Init( MPU_InitTypeDef *MPUx )
{
  uint8_t status = ERROR;
	uint8_t i;
	
  uint8_t MPU6555_InitData[MPU6555_InitRegNum][2] = 
	{
    {0x80, MPU6555_PWR_MGMT_1},     // Reset Device
    {0x01, MPU6555_PWR_MGMT_1},     // Clock Source
    {0x30, MPU6555_USER_CTRL},      // Set I2C_MST_EN, I2C_IF_DIS
    {0x00, MPU6555_PWR_MGMT_2},     // Enable Acc & Gyro
    {0x07, MPU6555_CONFIG},         // DLPF_CFG[2:0] = 111;
    {0x18, MPU6555_GYRO_CONFIG},    // +-2000dps
    {0x08, MPU6555_ACCEL_CONFIG},   // +-4G
    {0x48, MPU6555_ACCEL_CONFIG_2}, // Set Acc Data Rates
    {0x10, MPU6555_INT_PIN_CFG},    // Set INT_ANYRD_2CLEAR
    {0x10, MPU6555_INT_ENABLE},     // 
    {0x4D, MPU6555_I2C_MST_CTRL},   // I2C Speed 400 kHz
				
		{0x8C, MPU6555_I2C_SLV0_ADDR},  // AK8963_I2C_ADDR ( 7'b000_1100 )
    {0x00, MPU6555_I2C_SLV0_REG},   // AK8963_WIA ( 0x00 )
    {0x81, MPU6555_I2C_SLV0_CTRL},  // Enable
    {0x01, MPU6555_I2C_MST_DELAY_CTRL}
  };
	
  uint8_t AK8963_InitData[AK8963_InitRegNum][2] = {
    {0x01, AK8963_CNTL2},           // Reset Device
    {0x06, AK8963_CNTL1},           // Continuous measurement mode 2
  };

  MPU6555_InitData[5][0] = MPUx->MPU_Gyr_FullScale;       // MPU6555_GYRO_CONFIG
  MPU6555_InitData[4][0] = MPUx->MPU_Gyr_LowPassFilter;   // MPU6555_CONFIG
  MPU6555_InitData[6][0] = MPUx->MPU_Acc_FullScale;       // MPU6555_ACCEL_CONFIG
  MPU6555_InitData[7][0] = MPUx->MPU_Acc_LowPassFilter;   // MPU6555_ACCEL_CONFIG_2
  AK8963_InitData[1][0] |= MPUx->MPU_Mag_FullScale;

  for(i = 0; i < MPU6555_InitRegNum; i++) 
	{
    MPU9255_WriteReg(MPU6555_InitData[i][1], MPU6555_InitData[i][0]);
    delay_ms(10);
  }

  status = MPU9255_Check();
  if(status != SUCCESS)
    return ERROR;

  delay_ms(10);

  for(i = 0; i < AK8963_InitRegNum; i++)
	{
    MPU9255_Mag_WriteReg(AK8963_InitData[i][1], AK8963_InitData[i][0]);
    delay_ms(10);
  }
  MPU9255_WriteReg(MPU6555_I2C_SLV0_REG, AK8963_ST1);
	MPU9255_WriteReg(MPU6555_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
	MPU9255_WriteReg(MPU6555_I2C_SLV0_CTRL, 0x80 | 8);
	
  return SUCCESS;
}

void Sensor_Read(void)
{
	uint8_t ReadBuf[READ_9250_NUM*IMU_NUM] = {0};
	uint8_t i=0,j=0;

	
	MPU9255_ReadRegs(MPU6555_ACCEL_XOUT_H, ReadBuf, READ_9250_NUM);

	for(i=0;i<IMU_NUM;i++)   //IMU??
	{
		for(j=0;j<IMU_DEV_NUM;j++)
		{
			IMU_Data[i][j]=Byte16(int16_t, ReadBuf[j*2+i*READ_9250_NUM],  ReadBuf[j*2+1+i*READ_9250_NUM]);   //ACC.X
			
		/*if(j<7)
			{
				IMU_Data[i][j]=Byte16(int16_t, ReadBuf[j*2+i*READ_9250_NUM],  ReadBuf[j*2+1+i*READ_9250_NUM]);   //ACC.X
			}
			else
			{
				IMU_Data[i][j]=Byte16(int16_t, ReadBuf[(j+1)*2+i*READ_9250_NUM], ReadBuf[j*2+1+i*READ_9250_NUM]); 
			}
*/		
			
			
			if(IMU_Data[i][j]>32767) 
							
			IMU_Data[i][j]-=32767;   //??????
			
			else IMU_Data[i][j]+=32767;
			
			IMU_Pre_Data[i][j]=(float32_t)((IMU_Data[i][j]-32767)*Calibration_Coe[j]);
		}
	}
	for(j=0;j<IMU_DEV_NUM;j++)
	{
		IMU_Sol_Data[0][j]=(float32_t)(IMU_Pre_Data[IMU_U2][j]+IMU_Pre_Data[IMU_U3][j]+IMU_Pre_Data[IMU_U4][j]+IMU_Pre_Data[IMU_U5][j])*0.25f;
		IMU_Sol_Data[1][j]=(float32_t)(IMU_Pre_Data[IMU_U6][j]+IMU_Pre_Data[IMU_U7][j]+IMU_Pre_Data[IMU_U8][j]+IMU_Pre_Data[IMU_U9][j])*0.25f;
    Sensor_Original_Info.AD_DATA[j]=(IMU_Sol_Data[0][j]+IMU_Sol_Data[1][j])*0.5f;
  }
	
}

uint8_t MPU9255_Config_Check( void )
{
  uint8_t status = ERROR;

  MPU_InitTypeDef MPU_InitStruct;

  MPU9255_Config();
  delay_ms(100);

  MPU_InitStruct.MPU_Gyr_FullScale     = MPU_GyrFS_500dps;
  MPU_InitStruct.MPU_Gyr_LowPassFilter = MPU_GyrLPS_92Hz;
  MPU_InitStruct.MPU_Acc_FullScale     = MPU_AccFS_8g;
  MPU_InitStruct.MPU_Acc_LowPassFilter = MPU_AccLPS_93Hz;
  MPU_InitStruct.MPU_Mag_FullScale     = MPU_MagFS_16b;
  status = MPU9255_Init(&MPU_InitStruct);
  if(status != SUCCESS)
    return ERROR;

  return SUCCESS;
}




