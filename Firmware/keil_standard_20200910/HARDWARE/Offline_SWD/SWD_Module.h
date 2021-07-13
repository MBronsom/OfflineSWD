/**
 *@brief �����պ��һ���󹤳̣�һ��Ҫ�����ķǳ�����
 *
 */


#ifndef __SWD_MODULE_H
#define __SWD_MODULE_H

#include "stm32f10x.h"
#include "w25qxx.h" 
#include "memoryManage.h"


#if( HARDWARE_VERSION == 0 )  // mini�汾������
	#define SWD_RESET_PIN 							GPIOB->ODR.OutputData6_RW
	#define SWD_SWCLK_PIN								GPIOA->ODR.OutputData2_RW
	#define SWD_SWDIO_PIN_OUT						GPIOA->ODR.OutputData3_RW
	#define SWD_SWDIO_PIN_IN						GPIOA->IDR.InputData3_R

	// ����ģʽ������ģʽ��Ҫ��������ֹ��ȷ������Ȼ�ⲻ̫���ܣ�
	#define SWD_SWDIO_MODE_OUT					do{ \
																					GPIOA->CRL.InOutMode3_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;   \
																					GPIOA->CRL.PinConfig3_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL; \
																				}while(0); // ���ģʽ������û��д����ֵ
	#define SWD_SWDIO_MODE_IN 					do{	\
																					GPIOA->CRL.InOutMode3_RW = GPIO_CR_INOUNTMODE_INPUT;   \
																					GPIOA->CRL.PinConfig3_RW = GPIO_CR_PINCONFG_IN_PULLUP_PULLDOWN; \
																					GPIOA->ODR.OutputData3_RW = 1; \
																				}while(0);//����ģʽ������
#else


	#define SWD_RESET_PIN 							GPIOB->ODR.OutputData6_RW
	#define SWD_SWCLK_PIN								GPIOA->ODR.OutputData8_RW  // PA8���ţ���Ϊ�������
	#define SWD_SWDIO_PIN_OUT						GPIOB->ODR.OutputData4_RW
	#define SWD_SWDIO_PIN_IN						GPIOB->IDR.InputData4_R

	// ����ģʽ������ģʽ��Ҫ��������ֹ��ȷ������Ȼ�ⲻ̫���ܣ�
	#define SWD_SWDIO_MODE_OUT					do{ \
																					GPIOB->CRL.InOutMode4_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;   \
																					GPIOB->CRL.PinConfig4_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL; \
																				}while(0); // ���ģʽ������û��д����ֵ
	#define SWD_SWDIO_MODE_IN 					do{	\
																					GPIOB->CRL.InOutMode4_RW = GPIO_CR_INOUNTMODE_INPUT;   \
																					GPIOB->CRL.PinConfig4_RW = GPIO_CR_PINCONFG_IN_PULLUP_PULLDOWN; \
																					GPIOB->ODR.OutputData4_RW = 1; \
																				}while(0);//����ģʽ������

#endif																		
																			
#define SWD_TRANSFER_TIME_TO_STEP  3  // ����SWDÿһ�����������Դ����������ܵ�ÿ�����صĴ�������������趨�ģ���Ӧ�ñ��궨��
/**
 *  @B ���Է���ACK��ÿ������1��������ACK����Ҫ�ط�����֤�ȶ��ԣ�
 *  \warning STM32F1�ֲ��и��������෴����Ϊ��LSB�����ģ��������ս����һ���ġ�
 */

#define ACK_OK     0x1  // 001 : 1 0 0 
#define ACK_WAIT   0x2  // 010 : 0 1 0
#define ACK_FAULT  0x4  // 100 : 0 0 1

/**
 *  @B ��SWD�У�����LSB�ȴ��䣬����ʹ��LSB�ķ�ʽ����ת�����У�
 *  JTAG to SWD bit sequence, transmitted LSB first
 */
#define JTAG_TO_SWD_VALUE  0xE79E
																			
//#define JTAG_TO_SWD_VALUE  0xEDB6
//#define JTAG_TO_SWD_VALUE  0x6DB7
											
/**
 *  @B SWD���ͽ��λ�����Ҫ���������ԵĻ���������ó�enum������
 */																			
#define SWD_ERROR_OK         0
#define SWD_ERROR_PARITY     1
#define SWD_ERROR_WAIT       2   // �ڲ���F2��ʱ�򣬳����ˣ�����ʱ�����⣬����������ڣ�WAIT��ԭ�������б�Ҫ����WAIT״̬��
#define SWD_ERROR_FAULT      3	 // �������ҲҪС�ģ����п����Ƿ��ʴ������Բ�һ��OK����
#define SWD_ERROR_PROTOCOL   4

/* Address of DP read registers */
#define DP_IDCODE  0
#define DP_CTRL    1
#define DP_RESEND  2
#define DP_RDBUFF  3

/* Adressses of DP write registers */
#define DP_ABORT   0
#define DP_STAT    1
#define DP_SELECT  2

/* AHB-AP registers */
#define AP_CSW 0
#define AP_TAR 1
#define AP_DRW 3  // �������3��
#define AP_IDR 3  /* In bank 0xf : 0xF3 */

/* Powerup request and acknowledge bits in CTRL/STAT */
#define DP_CTRL_CDBGPWRUPREQ  ((u32)(1 << 28))
#define DP_CTRL_CDBGPWRUPACK  ((u32)(1 << 29))
#define DP_CTRL_CSYSPWRUPREQ  ((u32)(1 << 30))
#define DP_CTRL_CSYSPWRUPACK  ((u32)(1 << 31))



/***����Ϊ��һ���ο������еĶ���***/

// Cortex M3 Debug Registers (AHB addresses)
#define DDFSR   0xE000ED30      // Debug Fault StatusRegister
#define DHCSR   0xE000EDF0      // Debug Halting Control and Status Register
#define DCRSR   0xE000EDF4      // Debug Core Register Selector Register
#define DCRDR   0xE000EDF8      // Debug Core Register Data Register
#define DEMCR   0xE000EDFC      // Debug Exception and Monitor Control Register
#define AIRCR   0xE000ED0C      // The Application Interrupt and Reset Control Register

//  Cortex M3 Memory Access Port
#define MEMAP_BANK_0  0x00000000       // BANK 0 => CSW, TAR, Reserved, DRW
#define MEMAP_BANK_1  0x00000010       // BANK 1 => BD0, BD1, BD2, BD3

// SiM3 Chip Access Port (SiLabs specific Debug Access Port)
#define CHIPAP_BANK_0  0x0A000000      // BANK 0 => CTRL1, CTRL2, LOCK, CRC
#define CHIPAP_BANK_1  0x0A000010      // BANK 1 => INIT_STAT, DAP_IN, DAP_OUT, None
#define CHIPAP_BANK_F  0x0A0000F0      // BANK F => None, None, None, ID

// MEMAP register addresses
#define MEMAP_CSW     0x01
#define MEMAP_TAR     0x05
#define MEMAP_DRW_WR  0x0D
#define MEMAP_DRW_RD  0x0F

// CHIPAP register addresses
#define CHIPAP_CTRL1_WR     0x01
#define CHIPAP_CTRL2_WR     0x05
#define CHIPAP_ID_WR        0x0D
#define CHIPAP_ID_RD        0x0F

// ARM CoreSight DAP command values
#define DAP_IDCODE_RD           0x02
#define DAP_ABORT_WR            0x00
#define DAP_CTRLSTAT_RD         0x06
#define DAP_CTRLSTAT_WR         0x04
#define DAP_SELECT_WR           0x08
#define DAP_RDBUFF_RD           0x0E



typedef struct
{
  __IO u32 ACR;  
  __IO u32 KEYR;                
  __IO u32 OPTKEYR;   
  __IO u32 SR;                 
  __IO u32 CR;               
  __IO u32 OPTCR;  
  __IO u32 OPTCR1;
}F4_FLASH_TypeDef;

#define F4_FLASH               ((F4_FLASH_TypeDef *) ((u32)0x40023C00))


extern vu8 AutoDownloadEnable;
/***����Ϊ��һ���ο������еĶ���***/

void SWDInit( void );

void writeDP( u8 reg,u32 data );


u32 initDp( void );
u32 readApID( void );
void aapExtensionSequence( void );



u32 readMemoryFormStm32( void );

void haltTarget(void);
void runTarget(void);
void resetAndHaltTarget(void);
void runTarget(void);
void clearFlash( void );

void writeFlash( void );

u32 DownloadFlash( _FileInformation userfile );




void writeDP( u8 reg,u32 data );
void writeAP( u8 reg,u32 data);
void JTAG_To_SWD_Sequence( void );
u32 writeReg( u8 APnDPReg,u8 reg,u32 data,u8 ignoreAck);
u32 readReg( u8 APnDPReg,u8 reg, u32 *data );
void readAP( u8 reg,u32 *data );
void readDP( u8 reg,u32 * data );

#endif





