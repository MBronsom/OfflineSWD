/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Endpoint routines
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
#include "usb_lib.h"
#include "usb_bot.h"
#include "usb_istr.h"
#include <stdio.h>
#include "usb_desc.h"
#include "usart.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    : EP1 IN Callback Routine
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
  Mass_Storage_In();
}

/*******************************************************************************
* Function Name  : EP2_OUT_Callback.
* Description    : EP2 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP2_OUT_Callback(void)
{
  Mass_Storage_Out();
}

uint8_t buffer_out[64];
__IO uint32_t count_out = 0;
uint32_t count_in = 0;
char temp[60];
char temp_cnt = 0;


void EP6_OUT_Callback(void)
{
	int i = 0;

// tf("\r\nEP6_OUT_Callback\r\n");

  /* 清空上一次操作 */
  for(i=0; i<temp_cnt; i++)
  {
  	buffer_out[i] = 0;
  }

  /* 接收 */
  count_out = GetEPRxCount(ENDP6);
  PMAToUserBufferCopy(buffer_out, ENDP6_RXADDR, count_out);
  SetEPRxValid(ENDP6);

  temp_cnt = 	count_out;
	
  usart_sendCommand(buffer_out,temp_cnt);
}

/*******************************************************************************
* Function Name  : EP4_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP4_IN_Callback(void)
{
  count_in = 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

