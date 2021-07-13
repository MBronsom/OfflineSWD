/**
  ******************************************************************************
  * @file    mass_mal.c
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Medium Access Layer interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/

#include "w25qxx.h"
//#include "bsp_sdio_sdcard.h"
#include "mass_mal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Mass_Memory_Size[1];
uint32_t Mass_Block_Size[1];
uint32_t Mass_Block_Count[1];
__IO uint32_t Status = 0;
//extern SD_CardInfo SDCardInfo;	  //用于存储卡的信息

//SD_CardInfo mSDCardInfo;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : MAL_Init
* Description    : Initializes the Media on the STM32
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Init(uint8_t lun)
{
  uint16_t status = MAL_OK;

  switch (lun)
  {
    case 0:
		Mass_Memory_Size[0] = 8*1024*1024;
        Mass_Block_Size[0]  = 512 ;
        Mass_Block_Count[0] = Mass_Memory_Size[0] / Mass_Block_Size[0];
      break;
    case 1:

      break;
    default:
      return MAL_FAIL;
  }
  return status;
}
/*******************************************************************************
* Function Name  : MAL_Write
* Description    : Write sectors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Write(uint8_t lun, uint32_t Memory_Offset, uint32_t *Writebuff, uint16_t Transfer_Length)
{

  switch (lun)
  {
    case 0:{
		W25QXX_Write((u8*)Writebuff, (u32)Memory_Offset, Transfer_Length);
		break;
		}
    case 1:

      break;
    default:
      return MAL_FAIL;
  }
  return MAL_OK;
}

/*******************************************************************************
* Function Name  : MAL_Read
* Description    : Read sectors
* Input          : None
* Output         : None
* Return         : Buffer pointer
*******************************************************************************/
uint16_t MAL_Read(uint8_t lun, uint32_t Memory_Offset, uint32_t *Readbuff, uint16_t Transfer_Length)
{

  switch (lun)
  {
    case 0:
		W25QXX_Read((u8*)Readbuff, (u32)Memory_Offset, Transfer_Length);
      break;
//    case 1:
//      NAND_Read(Memory_Offset, Readbuff, Transfer_Length);
//      ;
//      break;
    default:
      return MAL_FAIL;
  }
  return MAL_OK;
}

/*******************************************************************************
* Function Name  : MAL_GetStatus
* Description    : Get status
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_GetStatus (uint8_t lun)
{
	switch(lun)
    {
		case 0:
			return MAL_OK;
		case 1:
			return MAL_OK; 
		default:
			return MAL_FAIL;
    } 
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

