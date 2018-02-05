/***************************************************************** 
*  @brief:       Storage the parameter  
*  @File:        Config.h  
*  @Project:     
*  @Author:      Liu Ning
*  @Date:        170613
*  @CopyRight:   
*  @Version:     V1.0
*  @Description: 
*****************************************************************/
#ifndef  __DATASTORAGE_H
#define  __DATASTORAGE_H

/*-select the device platform-*/
#if defined(STM32F429xx)
	#include "stm32f4xx_hal.h"
#elif defined(STM32F767xx)
  #include "stm32f7xx_hal.h"
#endif

#include "DataLink.h"

//see pdf 79
/*-flash sector-*/
#if defined(STM32F429xx)
  #define ADDR_FLASH_SECTOR_7      ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_8      ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_9      ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_10     ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_11     ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */
#elif defined(STM32F767xx)
  #define ADDR_FLASH_SECTOR_7      ((uint32_t)0x080C0000) /* Base @ of Sector 7, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_8      ((uint32_t)0x08100000) /* Base @ of Sector 8, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_9      ((uint32_t)0x08140000) /* Base @ of Sector 9, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_10     ((uint32_t)0x08180000) /* Base @ of Sector 10, 128 Kbytes */
  #define ADDR_FLASH_SECTOR_11     ((uint32_t)0x081C0000) /* Base @ of Sector 11, 128 Kbytes */

#endif

#define PAR_SECTOR_NUM  5     //the sector number used to storage parameter

#define PAR_SETTING   0        //the array number
#define PAR_CALI      1
#define PAR_DEV       2
#define PAR_USER      3
#define PAR_RES       4

typedef struct{
  uint32_t Setctor;
  uint32_t Start_Address;
  uint32_t End_Addresss;
  uint16_t CommandNum;
}Flash_Info;

extern Flash_Info Par_Storage_Info[PAR_SECTOR_NUM];

enum Data_Type_Info{
  Data_Storage_Flash=0,
  Data_Storage_RAM=1
};

/*read the data from internal flash*/
#define Read_Flash(__ADD__) (*(__IO uint8_t*)(__ADD__))


/*Define function*/
void DataStorage_Init(void); //initial the datastorage parameter
void Datastorage_Read(void);
void Storage_Setting_Par(void);
void Storage_Cali_Par(void);
void Storage_Dev_Par(void);
void Storage_User_Par(void);
void Storage_Res_Par(void);

void Read_Setting_Par(void);
void Read_Cali_Par(void);
void Read_Dev_Par(void);
void Read_User_Par(void);
void Read_Res_Par(void);


#endif

