/**
* @brief  ��IOģ��SWD�����ע����һ���󹤳̣�����Ҳ���պ������㣬Խ��Խ��
* @Author 
* @Data   2017.12.21
* \Attention ���κ�������ΪҪ�õ�ʵ����Ŀ�����еĳ������һ��Ҫ�ر�ã���ر�֤�ȶ���ǰ���£������ٶȣ��պ�϶�����SPIЭ��֮�����׵ģ���Ϊͨ��дLCDȷʵ����ģ����ٶ�
						���ҹ�ά������Ӳ����SPIȷʵ�ǲ��еġ�
*/

#include "SWD_Module.h"

vu8 slowModeEnable = 0;  // Ĭ�ϲ�ʹ�ܡ�0����ʹ�ܡ�1��ʹ�ܣ�
vu8 AutoDownloadEnable = 0;
void SWD_Delay( void )  
{
	for( vu32 i=0;i<=3;i++ );   // ���ڷ�����ֲ����
}
/**
 *  @B ��ʼ��SWD���ź���Ҫ�Ĺ��ܣ�
 *
 */

void SWDInit( void )
{
	{ // IO���ų�ʼ��
		{ // ���棺ʹ����PB3/PB4��������ǵ��Խӿڣ�Ĭ����������ܣ�������Ҫȥ����
			RCC->APB2ENR.AFIOClockEnable_RW = ENABLE;     // ʹ��ʱ��
			AFIO->MAPR.SerialWireJTAGConfiguration_W = 2; // ֻ����SWD���ţ�
		}
		
		if( HARDWARE_VERSION == LCD_VERSION )
		{
			RCC->APB2ENR.IOPBClockEnable_RW = ENABLE;
			SWD_SWDIO_MODE_OUT;  // ���ģʽ
			SWD_SWDIO_PIN_OUT = 1;  // ���Ϊ1
			
			GPIOB->CRL.InOutMode6_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
			GPIOB->CRL.PinConfig6_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL;  // RESET
			SWD_RESET_PIN = 1;   // �͵�ƽ��λ�����Բ���λ
			{  // SWCLK���ſ��ƣ���Ϊ�������������裬����ʹ�ÿ�©�����1��
				RCC->APB2ENR.IOPAClockEnable_RW = ENABLE;
				GPIOA->CRH.InOutMode8_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
				GPIOA->CRH.PinConfig8_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL;  // ���ݷ��֣�ʹ�ÿ�©���ţ�Ҳ��ʹ���0���������ͣ����ͨ�ž���̫�̣�����dpID�ʹ���͹��������׳���
				SWD_SWCLK_PIN = 1;   // Ĭ��Ϊ�ߵ�ƽ
				
				GPIOA->CRL.InOutMode0_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
				GPIOA->CRL.PinConfig0_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_OPEN_DRAIN;  // ���ſ�©���
				GPIOA->ODR.OutputData0_RW = 1;
				
				GPIOA->CRL.InOutMode1_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
				GPIOA->CRL.PinConfig1_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_OPEN_DRAIN;  // ���ſ�©���
				GPIOA->ODR.OutputData1_RW = 1;
			}
		}
		else
		{
			RCC->APB2ENR.IOPAClockEnable_RW = ENABLE;
			SWD_SWDIO_MODE_OUT;  // ���ģʽ
			SWD_SWDIO_PIN_OUT = 1;  // ���Ϊ1
			
			GPIOB->CRL.InOutMode6_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
			GPIOB->CRL.PinConfig6_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL;  // RESET
			SWD_RESET_PIN = 1;   // �͵�ƽ��λ�����Բ���λ
			{ // ��Ϊ����RESET���Ŷ�������λ�����Ի���ҪPB1
				GPIOB->CRL.InOutMode1_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
				GPIOB->CRL.PinConfig1_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL;  // RESET
				GPIOB->ODR.OutputData1_RW = 1;
			}
			{  // SWCLK���ſ��ƣ���Ϊ�������������裬����ʹ�ÿ�©�����1��
				RCC->APB2ENR.IOPAClockEnable_RW = ENABLE;
				GPIOA->CRL.InOutMode2_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
				GPIOA->CRL.PinConfig2_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL;  // ���ݷ��֣�ʹ�ÿ�©���ţ�Ҳ��ʹ���0���������ͣ����ͨ�ž���̫�̣�����dpID�ʹ���͹��������׳���
				SWD_SWCLK_PIN = 1;   // Ĭ��Ϊ�ߵ�ƽ
			}
		}
	}
}

void RAISE( u32 test )
{
	GPIOC->ODR.OutputData8_RW = test&0x1;  // 
	if( AutoDownloadEnable != ENABLE )
	{
		BeepCtrol(2,200);
	}
}

/**
 *  @B ��ʱ��Ŀ����Ҫ���ȶ����������е���ʱ�Ƚ϶࣡
 *  @Warning ��ת�������ʱ����Ҫע�⣬��Ϊ�Լ����˻�����,������Ҫע�⣡
 *  ��β��:SWCLK����Ϊ1
 */
void WRITE_BIT( u8 bit )
{
	if( slowModeEnable == ENABLE )
	{
		SWD_Delay();
		SWD_SWDIO_PIN_OUT = bit;
		SWD_Delay();
		SWD_SWCLK_PIN = 0;
		SWD_Delay();
		SWD_SWCLK_PIN = 1;
		SWD_Delay();
	}
	else
	{
		SWD_SWDIO_PIN_OUT = bit;
		SWD_SWCLK_PIN = 0;
		SWD_SWCLK_PIN = 1;
	}
}

u8 READ_BIT( void )
{
	u8 bit;
	if( slowModeEnable == ENABLE)
	{
		SWD_Delay( );
		SWD_SWCLK_PIN = 0;
		SWD_Delay( );
		bit = SWD_SWDIO_PIN_IN;
		SWD_Delay( );
		SWD_SWCLK_PIN = 1;
		SWD_Delay( );
	}
	else
	{
		SWD_SWCLK_PIN = 0;
		bit = SWD_SWDIO_PIN_IN;
		SWD_SWCLK_PIN = 1;
	}
	return bit;
}
/**
 *  @B �������ΪTurnAround
 *
 */
void SWCLK_CYCLE( void  )
{
	if( slowModeEnable == ENABLE)
	{
		SWD_Delay();
		SWD_SWCLK_PIN = 0;
		SWD_Delay();
		SWD_SWCLK_PIN = 1;
		SWD_Delay();
	}
	else
	{
		SWD_SWCLK_PIN = 0;
		SWD_SWCLK_PIN = 1;
	}
}

/**
 *  @B �������ΪTurnAround
 *
 */
void SWDIO_CYCLE( void )
{
	if( slowModeEnable == ENABLE)
	{
		SWD_Delay();
		SWD_SWDIO_PIN_OUT = 0;
		SWD_Delay();
		SWD_SWDIO_PIN_OUT = 1;
		SWD_Delay();
	}
	else
	{
		SWD_SWDIO_PIN_OUT = 0;
		SWD_SWDIO_PIN_OUT = 1;
	}
}
/**
 *  @B ��AP����DP�Ĵ������������
 *     reg:��Ȼֻ��4��(�ݶ���ȡֵ��Χ0-3)��  dataΪʲô��ָ�룬��Ϊ�������ܸı䴫��ֵ��Ҳ����ָ��ָ���ֵ
 *  ��β��:SWCLK����Ϊ1
 */
u32 readReg( u8 APnDPReg,u8 reg, u32 *data )
{
	u8 i = 0;
	u8 cb = 0;  // 
	u8 parity;  // У��ֵ
	u8 b = 0;				// ���ڶ�ACK��λ
	u8 ack = 0; // ACK��ֵ
	u8 ret = SWD_ERROR_OK;
	
	*data = 0;
	
	int _APnDPReg = (int) APnDPReg;
  int _read = (int) 1;  // ������ֵΪ1
	
	u8 A2 = reg & 0x01;
	u8 A3 = ( reg>>1 ) & 0x01;
	
	parity = ( _APnDPReg + _read + A2 + A3 ) & 0x01;
	
	SWD_SWDIO_MODE_OUT;   // ����Ϊ���ģʽ
//	SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // ����������Ϊ���ģʽ
	
	{  // ������������
		// �������е����⣺���ͺ󣬿��Կ���������SWCLK����Ϊ1:
		WRITE_BIT( 1 );
		WRITE_BIT( _APnDPReg );
		WRITE_BIT( _read );
		WRITE_BIT( A2 );
		WRITE_BIT( A3 );
		WRITE_BIT( parity );
		WRITE_BIT( 0 );
		WRITE_BIT( 1 );   // SWDIO = 1, SWCLK = 0, SWCLK = 1
	}
	{ // TurnAround
		{
			SWD_SWDIO_MODE_IN;   // ����Ϊ����ģʽ
//			SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_IN;  // ����������Ϊ����ģʽ
		}
		SWCLK_CYCLE();
	}
	{  // ��ACK
		for( i=0;i<3;i++ )
		{
			b = READ_BIT( );
			ack |= b<<i;
		}
		  // �����ҵ���⣬���ﲻ��Ӧ����һ��Trn��ô������ȴû��
	}
	{ // �ж�ACKλ
		if( ack == ACK_OK )
		{
			for( i=0;i<32;i++)
			{
				b = READ_BIT( );
				*data |= b<<i;
				if(b)
					cb = !cb;  // cb֮ǰ�Ѿ���ʼ����
			}
			parity = READ_BIT();  // ����ٶ�һ��У��λ
			                      // Ҫʹϵͳ�ȶ��Ļ����������������Ҫ
			if( cb == parity )
			{
				ret = SWD_ERROR_OK;  // ϵͳ����
			}
			else
			{
				ret = SWD_ERROR_PARITY;  // У����󣬼������Ϳ�����λ�������ԣ���Ȼ���������50%
			}
		}
		else if( ack == ACK_WAIT )
		{
			ret = SWD_ERROR_WAIT;
		}
		else if( ack == ACK_FAULT )
		{
			ret = SWD_ERROR_FAULT;
		}
		else
		{
			ret = SWD_ERROR_PROTOCOL;  //Э��������Ҫע����
		}
	}
	
	{ // Turnaround
		SWCLK_CYCLE();
	}
	{ // ����8��Idle״̬��ȷ������
		{
			SWD_SWDIO_MODE_OUT;   // ����Ϊ����ģʽ
//			SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // ����������Ϊ����ģʽ
		}
		for( i=0;i<8;i++ )
		{
			WRITE_BIT(0);
		}
	}
	return ret;    // ����ֵ
}


/**
 *  @B �����и�Ҫע�⣺���Ժ���Ack����Ϊ���������Ҫ�������ݼĴ���
 *
 */
u32 writeReg( u8 APnDPReg,u8 reg,u32 data,u8 ignoreAck)
{
	u8 ack = 0;
	u8 i;
	u8 parity = 0;
	u8 b;
	u8 ret = SWD_ERROR_OK;
	
	u8 _APnDPReg = APnDPReg;
	u8 _read = 0;
	
	u8 A2 = reg&0x1;
	u8 A3 = (reg>>1)&0x1;
	
	parity = ( _APnDPReg + _read +A2 + A3 )&0x1;  // ����У��ֵ
	
	SWD_SWDIO_MODE_OUT;   // ����Ϊ���ģʽ
//	SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // ����������Ϊ���ģʽ
	
	{  // ������������
		// �������е����⣺���ͺ󣬿��Կ���������SWCLK����Ϊ1:
		WRITE_BIT( 1 );
		WRITE_BIT( _APnDPReg );
		WRITE_BIT( _read );
		WRITE_BIT( A2 );
		WRITE_BIT( A3 );
		WRITE_BIT( parity );
		WRITE_BIT( 0 );
		WRITE_BIT( 1 );   // SWDIO = 1, SWCLK = 0, SWCLK = 1 ���������������Ϊ1�ģ��ֲ�˵�ǲ������������������������������ϻ�Ϊ1��
	}
	{
		{ // ����һ���ر���������⣺�����������һ�䣬�ŵ�SWCLK_CYCLE();֮��ͻ���֣�ACK���󣬱ȽϷѽ⣡
			// ���ܵ�ԭ�򣺾��Ǵӻ����ʱ��Ҫ����SWDIO�����Ƿ�����������������
			// ��õĽ���취������ͨ����DP�Ĵ����ҵ�������⣡
			SWD_SWDIO_MODE_IN; 
//			SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_IN;
		}
		SWCLK_CYCLE();
	}
	{
		{ 
			
			// ��ACK
			for( i=0;i<3;i++ )
			{
				b = READ_BIT( );
				ack |= b<<i;
			}
				// �����ҵ���⣬���ﲻ��Ӧ����һ��Trn��ô������ȴû�У�������ֶ���û�е��ˣ�����д���еģ�д�������棩
		}
	}
	{ // 
		if( (ack == ACK_OK) || ignoreAck )
		{
			
			{ // ����Ϊ���
				SWD_SWDIO_MODE_OUT;   // ����Ϊ���ģʽ
//				SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // ����������Ϊ���ģʽ
			}
			SWCLK_CYCLE();  // д��ʱ��Ҫ��Trnһ�Ρ����Ƕ���ʱ��û���ڴ������濴����
			parity = 0;    // ������������⣬���Ҹ�parity��ʱ�򣬳��ֿ��Զ�״̬��˵����������ֵ���ڼ�����󣡣�����ǲ�Ӧ�õġ����Ҫ��飡
			for( i=0;i<32;i++ )
			{
				b = ( data >> i)&0x1;
				WRITE_BIT(b);
				if(b)
					parity = !parity;
			}
			WRITE_BIT(parity);
		}
		else if( ack == ACK_WAIT )
		{
			ret = SWD_ERROR_WAIT;
		}
		else if( ack == ACK_FAULT )
		{
			ret = SWD_ERROR_FAULT;
		}
		else
		{
			ret = SWD_ERROR_PROTOCOL;  //Э��������Ҫע����
		}
		
		{ // ����8��Idle״̬��ȷ�����䣬��������Ż���
			{  // �����ʵ�Ƕ���ģ�һֱ������ģ������Ҷ��������Դ����������ˡ�
				SWD_SWDIO_MODE_OUT;   // ����Ϊ����ģʽ
//				SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // ����������Ϊ����ģʽ
			}
			for( i=0;i<8;i++ )
			{
				WRITE_BIT(0);
			}
		}
	}
	return ret;
}


/**
 *  @B �ϵ�ִ��JTAGתSWD�����к�����
 *
 */
void JTAG_To_SWD_Sequence( void )
{
  int i;
  int b;
  
	SWD_SWDIO_MODE_OUT;   // ����Ϊ���ģʽ
//	SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // ����������Ϊ���ģʽ
	{
		SWD_SWDIO_PIN_OUT = 1;   // �����Ϊ1
		for( i=0;i<80;i++ )
		{
				SWCLK_CYCLE();
		}
	}
	for( i=0;i<16;i++ )
	{
		b = ( JTAG_TO_SWD_VALUE>>i )&0x1;
		WRITE_BIT(b);
	}
	{
		SWD_SWDIO_PIN_OUT = 1;
		for( i=0;i<60;i++ )
		{
				SWCLK_CYCLE();
		}
	}
	{ // ִ��16����������
		SWD_SWDIO_PIN_OUT = 0;
		for( i=0;i<16;i++ )
		{
			SWCLK_CYCLE();
		}
		SWD_SWDIO_PIN_OUT = 1;  // ʵ���У���Ϊ���ˣ���һ�䵼��ֻ������F1��F4/F2���ǳ������⡣
	}
		
}


/**
 *  @B дAP�Ĵ������ڵ�ǰѡ���APBANK��
 *
 */


void writeAP( u8 reg,u32 data)
{
	u8 forFault = 0;
	forWait:
	{
		u8 swdStatus;
		u8 retry = 1;  // ��������Դ����������պ�Ӧ�����������������������һ��Ҫ��ȡ������
		do
		{
			swdStatus = writeReg(1,reg,data,FALSE);
			retry--;
		}while( (swdStatus == SWD_ERROR_WAIT) && retry>0 );
		if( swdStatus != SWD_ERROR_OK )
		{
			if( swdStatus == SWD_ERROR_WAIT )
			{
				goto forWait;  // �ڲ���F2��ʱ�򣬳����˲���ʱ��������⣬���Բ������������
			}
			else if( swdStatus == SWD_ERROR_FAULT )
			{
				if( forFault == 0 ) // ���Դ�������
				{
					forFault ++;
					writeDP( DP_ABORT,0x1F);
					goto forWait; 
				}
				else
				{
					RAISE( swdStatus );
				}
			}
			else
			{
				RAISE( swdStatus );  // ���ݲ�ѯԴ�룬��������Ĳ�����ԱȽ��鷳����Ҫ�Լ��պ����¶��壡
			}
		}
	}
}

/**
 *  @B дDP�Ĵ���
 *
 */
void writeDP( u8 reg,u32 data )
{
	u8 forFault = 0;
	forWait:
	{
		u8 swdStatus;
		u8 retry = 10;  // ���Դ���
		do
		{
			swdStatus = writeReg(0,reg,data,FALSE);
			retry--;
		}while( (swdStatus == SWD_ERROR_WAIT) && retry>0 );
		if( swdStatus != SWD_ERROR_OK )
		{
			if( swdStatus == SWD_ERROR_WAIT )
				{
					goto forWait;  // �ڲ���F2��ʱ�򣬳����˲���ʱ��������⣬���Բ������������
				}
				else if( swdStatus == SWD_ERROR_FAULT )
				{
					if( forFault == 0 ) // ���Դ�������
					{
						forFault ++;
						writeDP( DP_ABORT,0x1F);
						goto forWait; 
					}
					else
					{
						RAISE( swdStatus );
					}
				}
				else
				{
					RAISE( swdStatus ); 
				}
		}
	}
}

/**
 *  @B дDP�Ĵ������Ǻ���ACKλ�������Ȼ�ǳ���֮��Ĳ�����
 *
 */
void writeDPIgnoreAck( u8 reg,u32 data )
{
	u8 forFault = 0;
	forWait:
	{
		u8 swdStatus;
		u8 retry = 10;  // ���Դ���
		do
		{
			swdStatus = writeReg(0,reg,data,TRUE);
			retry--;
		}while( (swdStatus == SWD_ERROR_WAIT) && retry>0 );
		if( swdStatus != SWD_ERROR_OK )
		{
			if( swdStatus == SWD_ERROR_WAIT)
				{
					goto forWait;  // �ڲ���F2��ʱ�򣬳����˲���ʱ��������⣬���Բ������������
				}
				else if( swdStatus == SWD_ERROR_FAULT )
				{
					if( forFault == 0 ) // ���Դ�������
					{
						forFault ++;
						writeDP( DP_ABORT,0x1F);
						goto forWait; 
					}
					else
					{
						RAISE( swdStatus );
					}
				}
				else
				{
					RAISE( swdStatus );  // �������ر���Ҫ��
				}
		}
	}
}

/**
 *  @B ��AP
 *
 */
void readAP( u8 reg,u32 *data )
{
	u8 forFault = 0;
	forWait:
	{
		u8 swdStatus;
		u8 retry = 10;  // ���Դ���
		do
		{
			swdStatus = readReg( 1,reg,data );
			retry--;
		}while( (swdStatus == SWD_ERROR_WAIT) && retry>0 );
		if( swdStatus != SWD_ERROR_OK )
		{
			if( swdStatus == SWD_ERROR_WAIT)
				{
					goto forWait;  // �ڲ���F2��ʱ�򣬳����˲���ʱ��������⣬���Բ������������
				}
				else if( swdStatus == SWD_ERROR_FAULT )
				{
					if( forFault == 0 ) // ���Դ�������
					{
						forFault ++;
						writeDP( DP_ABORT,0x1F);
						goto forWait; 
					}
					else
					{
						RAISE( swdStatus );
					}
				}
				else
				{
					RAISE( swdStatus );  // �������ر���Ҫ��
				}
		}
	}
}

/**
 *  @B ��DP
 *
 */
void readDP( u8 reg,u32 * data )
{
	u8 forFault = 0;
	forWait:
	{
		u8 swdStatus;
		u8 retry = 10;  // ���Դ���
		do
		{
			swdStatus = readReg( 0,reg,data );
			retry--;
		}while( (swdStatus == SWD_ERROR_WAIT) && retry>0 );
		if( swdStatus != SWD_ERROR_OK )
		{
			if( swdStatus == SWD_ERROR_WAIT)
				{
					goto forWait;  // �ڲ���F2��ʱ�򣬳����˲���ʱ��������⣬���Բ������������
				}
				else if( swdStatus == SWD_ERROR_FAULT )
				{
					if( forFault == 0 ) // ���Դ�������
					{
						forFault ++;
						writeDP( DP_ABORT,0x1F);
						goto forWait; 
					}
					else
					{
						RAISE( swdStatus );
					}
				}
				else
				{
					RAISE( swdStatus );  // �������ر���Ҫ��
				}
		}
	}
}


/**
 *  @B ��ʼ��DP���ȷ���һ��ת�����У�Ȼ�����IDCode��ֵ��
 *
 */
u32 initDp( void )
{
	u32 dpId;
	JTAG_To_SWD_Sequence(  );
	readDP(DP_IDCODE,&dpId);  // 
	
	//Debug power up request�����Ӧ���Ǵ򿪵�������ĵ�Դ��������ԣ�
	writeDP( DP_CTRL, DP_CTRL_CSYSPWRUPREQ | DP_CTRL_CDBGPWRUPREQ );
	
	// Wait until we receive powerup ACK 
	int retry = 300;
	u32 status;
	while( retry>0 )
	{
		readDP(DP_CTRL,&status);
		// ����������ʲô��˼��
		if ( ( status & ( DP_CTRL_CDBGPWRUPACK | DP_CTRL_CSYSPWRUPACK ) )
      == ( DP_CTRL_CDBGPWRUPACK | DP_CTRL_CSYSPWRUPACK ) )
    {
      break;
    }
    retry--;
	}
	
	/* Throw error if we failed to power up the debug interface */
  if ( ( status & ( DP_CTRL_CDBGPWRUPACK | DP_CTRL_CSYSPWRUPACK ) )
    != ( DP_CTRL_CDBGPWRUPACK | DP_CTRL_CSYSPWRUPACK ) )
  {
//    RAISE( SWD_ERROR_DEBUG_POWER );  // ��������ù���
		RAISE( status );
  }
  
  /* Select first AP bank */
  writeDP( DP_SELECT, 0x00 );
  
  return dpId;  // Ӧ����1BA01477(F4������ֵΪ��2BA01477)  F2����Ҳ�ǣ�2BA01477
}



/**
 *  @B ������������EMF32��Ƭ���ģ�������һ���Ĳο���ֵ����Ҳ�ܶ��������������ǲ�֪����ʲô���壡
 */
u32 readApID( void )
{
	u32 apId;
	/* Select last AP bank */
  writeDP( DP_SELECT, 0xf0 );  // дDP�Ĵ����е�APBBANKSELλΪ0xF
//	writeDP( DP_SELECT, 0x08000003 );
  
  /* Dummy read AP ID */
  readAP( AP_IDR, &apId );
  
  /* Read AP ID */
  readDP( DP_RDBUFF, &apId );
  
  /* Select first AP bank again */
  writeDP( DP_SELECT, 0x00 );
  
  return apId; // EMF�ֲ���˵�����ֵӦ���ǣ�0x24770011��F4������ֵ���������������ʵ�ʶ�������0x14770011��F103������֪������ʲô���壡
}

/**
 *  @B ��������
 *
 */
/**********************************************************
 * Sends the AAP Window Expansion Sequence. This sequence
 * should be sent on SWDIO/SWCLK when reset is held low.
 * This will cause the AAP window to be 255 times longer
 * when reset is released. 
 **********************************************************/
void aapExtensionSequence( void )
{
  int i;
  
	SWD_RESET_PIN = 0;
	
  SWD_SWCLK_PIN = 1;
  
  for ( i = 0; i < 4; i++ )
  {
    SWDIO_CYCLE( );
  }
  
  SWD_SWCLK_PIN = 0;
  
  for ( i = 0; i < 4; i++ )
  {
    SWDIO_CYCLE( );
  }
  
  SWD_SWCLK_PIN = 0;
	
	SWD_RESET_PIN = 1;
}



/* �����濪ʼ��������ֲ������һ������--��SRAM�б�� �����в���Ĳο���ֵ */
/**
 *  @B ���ڳ������Ҫֹͣ���к���дFLash���������������������
 *
 */
void connect_and_halt_core( void ) 
{
	u32 rw_data;
	rw_data = CHIPAP_BANK_F;
	
	
	if( rw_data != 0x2430002 )
	{
		return;
	}
	
	
}

/*���ϣ�������ֲ������һ������--��SRAM�б�� �����в���Ĳο���ֵ */

/* �����濪ʼ�����Լ��Ĳ��Գ��� */
u32 readMemoryFormStm32( void )
{
	u32 apId = 0;
	vu32 address = 0x08000000;
	/* Select last AP bank */
//	while(1)
	{  // ��������ĳ�����ԣ��ٶȴ�Լ�ǣ�4MHZ����(3.7MHZ)����һ�Σ����ǱȽϸ����ġ�
		writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
	//	writeDP( DP_SELECT, 0x08000003 );
		
		writeAP( AP_CSW,0x2);   // ����Ϊ32bit
		
		writeAP( AP_TAR,address);   // дĿ���ַ
		readAP( AP_DRW,&apId);
		
		/* Read AP ID */
		readDP( DP_RDBUFF, &apId );
		address++;
  }
  
  return apId;
}

/**
 *  @B ����
 *
 */

void testFlashChearAndWrite( void )
{
	{	// ���Flashλ
		
	}
	{ // дFlash
		
	}
}


#define CMX_AIRCR 0xE000ED0C
#define RUN_CMD_CRJ 0x05FA0007  // ����Ǵӣ�DAP����ȡ�����ģ���֪���в��У���

#define CM3_DHCSR 0xE000EDF0
#define CM3_DEMCR 0xE000EDFC
/* Write these to DHCSR */
#define RUN_CMD  0xA05F0001
#define STOP_CMD 0xA05F0003
#define STEP_CMD 0xA05F0005
#define DEBUG_EVENT_TIMEOUT 200
/**
 *  @B Halting the MCU
 *
 */
void haltTarget(void)
{
	u32 tmp;
	writeAP(AP_TAR, CM3_DEMCR);
	readAP(AP_DRW, &tmp);
	readDP(DP_RDBUFF, &tmp);
	
	
	writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
  int timeout = DEBUG_EVENT_TIMEOUT;
  writeAP(AP_TAR, CM3_DHCSR);
  writeAP(AP_DRW, STOP_CMD);
	
	writeAP(AP_TAR, CM3_DEMCR);
  writeAP(AP_DRW, 0x01000401);  // Write 1 to bit VC_CORERESET in DEMCR. This will enable halt-on-reset
	

  uint32_t dhcrState;
  do {
    writeAP(AP_TAR, CM3_DHCSR);
    readAP(AP_DRW, &dhcrState);
    readDP(DP_RDBUFF, &dhcrState);
    timeout--;
  } while ( !(dhcrState & CoreDebug_DHCSR_S_HALT_Msk) && timeout > 0 ); 
  
  if ( !(dhcrState & CoreDebug_DHCSR_S_HALT_Msk) ) {
//		RAISE(dhcrState);
  }
	{ // �ֲ���Ľ����ǣ�ʹ��һ���ϵ㣡��
	}
}

/**
 *  @B ���Գɹ��������������򣡵���Ҫע��CSW��ֵ����Ҫ����Ϊ��0x23000002����ʱ�����׺��壬���Ҫע�⣡�պ���������
 *
 */
void runTarget(void)
{
	uint32_t i;
	writeAP( AP_CSW,0x23000002); 
	
	writeAP(AP_TAR, CM3_DEMCR);
  writeAP(AP_DRW, 0x01000400);  // ���⸴λͣ��Reset��
	for (i=0; i<100; i++);
  writeAP(AP_TAR, CM3_DHCSR);
  writeAP(AP_DRW, RUN_CMD);
	
	
	
	for (i=0; i<100; i++);
	writeAP(AP_TAR, CMX_AIRCR);
  writeAP(AP_DRW, RUN_CMD_CRJ);
	for (i=0; i<100; i++);
	writeAP(AP_TAR, CMX_AIRCR);
  writeAP(AP_DRW, RUN_CMD_CRJ);
}



/**********************************************************
 * Reads one word from internal memory
 * 
 * @param addr 
 *    The address to read from
 * 
 * @returns 
 *    The value at @param addr
 **********************************************************/
uint32_t readMem(uint32_t addr)
{
  uint32_t ret;
  writeAP(AP_TAR, addr);
  readAP(AP_DRW, &ret);
  readDP(DP_RDBUFF, &ret);
  return ret;
}
/**********************************************************
 * Writes one word to internal memory
 * 
 * @param addr 
 *    The address to write to 
 *
 * @param data
 *    The value to write
 * 
 * @returns 
 *    The value at @param addr
 **********************************************************/
void writeMem(uint32_t addr, uint32_t data)
{
  writeAP(AP_TAR, addr);
  writeAP(AP_DRW, data);
}
/**********************************************************
 * Resets the target CPU by using the AIRCR register. 
 * The target will be halted immediately when coming
 * out of reset. Does not reset the debug interface.
 **********************************************************/
void resetAndHaltTarget(void)
{
  uint32_t dhcsr;
  int timeout = DEBUG_EVENT_TIMEOUT;
  
  /* Halt target first. This is necessary before setting
   * the VECTRESET bit */
  haltTarget();
  
  /* Set halt-on-reset bit */
  writeMem(DEMCR, CoreDebug_DEMCR_VC_CORERESET_Msk);
  
  /* Clear exception state and reset target */
  writeAP(AP_TAR, AIRCR);
  writeAP(AP_DRW, (0x05FA << SCB_AIRCR_VECTKEY_Pos) |
                  SCB_AIRCR_VECTCLRACTIVE_Msk |
                  SCB_AIRCR_VECTRESET_Msk);
    
  /* Wait for target to reset */
  do { 
    TimeDelayOfSoftAtMs(1);
    timeout--;
    dhcsr = readMem(DHCSR);
  } while ( dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk );
  
  
  /* Check if we timed out */
  dhcsr = readMem(DHCSR);
  if ( dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk ) 
  {
//    RAISE(SWD_ERROR_TIMEOUT_WAITING_RESET);
		RAISE(1);
  }
  
  /* Verify that target is halted */
  if ( !(dhcsr & CoreDebug_DHCSR_S_HALT_Msk) ) 
  {
//    RAISE(SWD_ERROR_TARGET_NOT_HALTED);
		RAISE(1);
  }
}



/**
 *  @B ���STM32 Flash����
 *
 */
void clearAllFlash( void )
{
	OS_ERR err;
	writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
	
  writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
  writeAP(AP_DRW, FLASH_KEY1);
	
	writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
  writeAP(AP_DRW, FLASH_KEY2);
	
	{ // �������
			writeAP(AP_TAR, (u32)&FLASH->SR.All ) ;
			writeAP(AP_DRW, _0b00110100);
	}
	
	writeAP( AP_CSW,0x00000002);   // ��
	u32 readWord0 = 0;
	do
	{
		writeAP(AP_TAR, (u32)&FLASH->SR.All);
		readAP(AP_DRW,&readWord0);
		readDP(DP_RDBUFF,&readWord0);
	}while(readWord0&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
	
	writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
	writeAP( AP_TAR, (u32)&FLASH->CR.All );
  writeAP( AP_DRW, _0b00000100 );  // ע�⣺������ʱ��Ҫ�ֿ�д��
	
	writeAP( AP_TAR, (u32)&FLASH->CR.All );
  writeAP( AP_DRW, _0b01000100 );  // ��д���ȫƬ����д��ʼλ����
	
	writeAP( AP_CSW,0x00000002);   // ��
	u32 readWord1 = 0;
	do
	{
		writeAP(AP_TAR, (u32)&FLASH->SR.All);
		readAP(AP_DRW,&readWord1);
		readDP(DP_RDBUFF,&readWord1);
	}while(readWord1&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
	{ // ���ݿͻ���Ӧ��ȫƬ�����п��ܴ���û����ɵĵط������Ա����ϸ�İ��ղ�������������
		// 
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while( ( readWord1&_0b00100000 ) != _0b00100000);
	}
	{ // �������
			writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
			writeAP(AP_TAR, (u32)&FLASH->SR.All);
			  writeAP(AP_DRW, _0b00110100);
	}
	OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err); 
}


/**
 *  @B дFlash������������ԣ�
 *
 */

void writeFlash( void )
{
	
}


/**
 *  @B F0ϵ�еı�������ѡ�ע�⣺F0ϵ�е�FLASH���Ҳ��16bitһ�α�������F1�Ĳ�����һ���ġ�
    @warning ʵ���з���F0�ļĴ��������F1������һ���ģ�����ѡ����F1�ļĴ������в�����
 *  
 */
void stm32f0Protection( u8 readProtectionEnable,u8 writeProtectionEnable )
{
	u32 readWord1 = 0;
//	if( (readProtectionEnable == 1) || (writeProtectionEnable == 1 ))  // 
//	{
		{  // ����Flash����
			writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
			
			writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
			writeAP(AP_DRW, FLASH_KEY1);
			
			writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
			writeAP(AP_DRW, FLASH_KEY2);
		}
		{
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
		}		
		{ // ����OptionsByte����
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY1);
			
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY2);
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00100000 + 0x200 );  // ���ѡ���ֽڣ�ע�⣺��ʱ��������Ȼ�ڣ�д������������û��ֽڱ����
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b01100000 + 0x200 );  // ��ʼ����
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // æʱ��ס
		}
		{ // �Ȳ�Ҫ��λ������
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 );  // �ָ��ֳ�
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00010000 + 0x200 );  // ѡ���ֽڱ��ʹ�ܣ�
		}
		{
			if( readProtectionEnable == ENABLE )  // Ĭ��״̬�¾��Ƕ�����ʹ�ܵģ�����Ϊ����ˣ���
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0x00);  // 0x00Ҳ�Ƕ�����
			}
			else
			{
				writeAP( AP_CSW,0x23000001 ); // ����Ϊ16bit,д������Flash������u16��ʽ�����������⣡��
				writeAP( AP_TAR, (u32)&OB->RDP );
				if( ((u32)&OB->RDP)%4 != 0 )
				{
					writeAP( AP_DRW, 0xAA<<16 );  // �������������ֵΪ0xAAʱ���ض�������
				}
				else
				{
					writeAP( AP_DRW, 0xAA);  // ���������
				}
			}
			if( writeProtectionEnable == ENABLE )
			{
				writeAP( AP_CSW,0x23000001 ); // ����Ϊ16bit,д������Flash������u16��ʽ�����������⣡��
				writeAP( AP_TAR, (u32)&OB->WRP0 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP1 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP2 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP3 );
				writeAP( AP_DRW, 0x00 );
				
			}
			else // ֮ǰ�Ѿ�����ˣ����Բ���Ҫ�ܣ�
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0xFFFFFFFF);  // ���������
			}
			
			writeAP( AP_CSW,0x23000002 ); // �ָ�Ϊ32λ
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // æʱ��ס
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 );  // �ָ��ֳ�
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b10000000);  	// ��������
		}
}
/**
 *  @B ������ʹ��
 *  ���棺����ѡ���ֽ�Ҳ����ͨ��u16�͵ķ��ʣ������������ܳɹ�������
 *  
 */

void stm32f1Protection( u8 readProtectionEnable,u8 writeProtectionEnable )
{
	u32 readWord1 = 0;
//	if( (readProtectionEnable == 1) || (writeProtectionEnable == 1 ))  // 
//	{
		{  // ����Flash����
			writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
			
			writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
			writeAP(AP_DRW, FLASH_KEY1);
			
			writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
			writeAP(AP_DRW, FLASH_KEY2);
		}
		{
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
		}		
		{ // ����OptionsByte����
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY1);
			
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY2);
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00100000 + 0x200 );  // ���ѡ���ֽڣ�ע�⣺��ʱ��������Ȼ�ڣ�д������������û��ֽڱ����
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b01100000 + 0x200 );  // ��ʼ����
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // æʱ��ס
		}
		{ // �Ȳ�Ҫ��λ������
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 );  // �ָ��ֳ�
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00010000 + 0x200 );  // ѡ���ֽڱ��ʹ�ܣ�
		}
		{
			if( readProtectionEnable == ENABLE )  // Ĭ��״̬�¾��Ƕ�����ʹ�ܵģ�
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0x00);  // 0x00Ҳ�Ƕ�����
			}
			else
			{
				writeAP( AP_CSW,0x23000001 ); // ����Ϊ16bit,д������Flash������u16��ʽ�����������⣡��
				writeAP( AP_TAR, (u32)&OB->RDP );
				if( ((u32)&OB->RDP)%4 != 0 )
				{
					writeAP( AP_DRW, RDP_Key<<16 );  // ���������
				}
				else
				{
					writeAP( AP_DRW, RDP_Key);  // ���������
				}
			}
			if( writeProtectionEnable == ENABLE )
			{
				writeAP( AP_CSW,0x23000001 ); // ����Ϊ16bit,д������Flash������u16��ʽ�����������⣡��
				writeAP( AP_TAR, (u32)&OB->WRP0 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP1 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP2 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP3 );
				writeAP( AP_DRW, 0x00 );
				
			}
			else // ֮ǰ�Ѿ�����ˣ����Բ���Ҫ�ܣ�
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0xFFFFFFFF);  // ���������
			}
			
			writeAP( AP_CSW,0x23000002 ); // �ָ�Ϊ32λ
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // æʱ��ס
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 + 0x2000);  // �ָ��ֳ������һ����0x2000������GD32�ǣ�ǿ�����¼���ѡ���ֽڣ�			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b10000000 + 0x2000);  	// ��������
		}
}


void stm32f3Protection( u8 readProtectionEnable,u8 writeProtectionEnable )
{
	u32 readWord1 = 0;
//	if( (readProtectionEnable == 1) || (writeProtectionEnable == 1 ))  // 
//	{
		{  // ����Flash����
			writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
			
			writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
			writeAP(AP_DRW, FLASH_KEY1);
			
			writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
			writeAP(AP_DRW, FLASH_KEY2);
		}
		{
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
		}		
		{ // ����OptionsByte����
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY1);
			
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY2);
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00100000 + 0x200 );  // ���ѡ���ֽڣ�ע�⣺��ʱ��������Ȼ�ڣ�д������������û��ֽڱ����
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b01100000 + 0x200 );  // ��ʼ����
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // æʱ��ס
		}
		{ // �Ȳ�Ҫ��λ������
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 );  // �ָ��ֳ�
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00010000 + 0x200 );  // ѡ���ֽڱ��ʹ�ܣ�
		}
		{
			if( readProtectionEnable == ENABLE )  // Ĭ��״̬�¾��Ƕ�����ʹ�ܵģ�
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0x00);  // 0x00Ҳ�Ƕ�����
			}
			else
			{
				writeAP( AP_CSW,0x23000001 ); // ����Ϊ16bit,д������Flash������u16��ʽ�����������⣡��
				writeAP( AP_TAR, (u32)&OB->RDP );
				if( ((u32)&OB->RDP)%4 != 0 )
				{
					writeAP( AP_DRW, 0xAA<<16 );  // ���������
				}
				else
				{
					writeAP( AP_DRW, 0xAA);  // ���������
				}
			}
			if( writeProtectionEnable == ENABLE )
			{
				writeAP( AP_CSW,0x23000001 ); // ����Ϊ16bit,д������Flash������u16��ʽ�����������⣡��
				writeAP( AP_TAR, (u32)&OB->WRP0 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP1 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP2 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP3 );
				writeAP( AP_DRW, 0x00 );
				
			}
			else // ֮ǰ�Ѿ�����ˣ����Բ���Ҫ�ܣ�
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0xFFFFFFFF);  // ���������
			}
			
			writeAP( AP_CSW,0x23000002 ); // �ָ�Ϊ32λ
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // æʱ��ס
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 );  // �ָ��ֳ�
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b10000000);  	// ��������
		}
}

/**
 *  @B F2ϵ�еĶ�д����������ע�����еĲ�����ֱ�Ӵ�F4��ֲ�����ģ���Ҫ��ֵ���֤��
 *  
 */

u32 stm32f2Protection( u8 readProtectionEnable,u8 writeProtectionEnable )
{
//	u32 readWord1 = 0;
//	
//	u32 protectValue0 = 0; // �Ĵ���OPTCR��ֵ
////	u32 protectValue1 = 0; // �Ĵ���OPTCR1��ֵ
//	
////	u32 F4FlashMemorySize = 0;  // �ڲ������������ͱ�������ʱ������Ҫ֪�����С����������û�ж����ֵ��
////		{ // ���Flash�Ĵ�С��
////			u32 apId = 0;
////			writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
////			writeAP( AP_CSW,0x2 );   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
////			writeAP( AP_TAR,(u32)(0x1FFF7A22));   // дĿ���ַ
////			readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
////			readDP( DP_RDBUFF, &apId );
////			F4FlashMemorySize = apId>>16;
////		}
//		writeDP( DP_ABORT,0x1F);
//	
//	if( readProtectionEnable == ENABLE )  // ʹ�ܶ�����
//	{
//		if( writeProtectionEnable == ENABLE) // ʹ��д����
//		{
//			protectValue0 = 0x0000FFED - 1;  // ʹ��д������RDP��0xFF��ʹ��һ��������
////			protectValue1 = 0x00000000;  // ʹ��д����
//		}
//		else
//		{
//			protectValue0 = 0x0FFFFFED - 1;
////			protectValue1 = 0x0FFF0000;
//		}
//	}
//	else
//	{
//		if( writeProtectionEnable == ENABLE)
//		{
//			protectValue0 = 0x0000AAED - 1;
////			protectValue1 = 0x00000000;
//		}
//		else  // ȫ������ʹ��
//		{
//			protectValue0 = 0x0FFFAAED - 1;
////			protectValue1 = 0x0FFF0000;
//		}
//	}
//	
//	{
//			{  // ����Flash����
//				writeAP( AP_CSW,0x23000002);   // 32λд����
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0x45670123 );
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0xCDEF89AB );
//				
//				writeAP( AP_CSW,0x00000002);   // ��
//				u32 readWord0 = 0;
//				do
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//					readAP(AP_DRW,&readWord0);
//					readDP(DP_RDBUFF,&readWord0);
//				}while((readWord0>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
//			}
//			{  // ����ѡ���ֽ�Flash����
//				writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
//				
//				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
//				writeAP(AP_DRW, 0x08192A3B);
//				
//				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
//				writeAP(AP_DRW, 0x4C5D6E7F);
//			}
//			{  // ����ǰҪȷ��û��Flash���в���
//				do
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//					readAP(AP_DRW,&readWord1);
//					readDP(DP_RDBUFF,&readWord1);
//				}while( (readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
//			}	
//			{
//				writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
//				writeAP(AP_DRW, protectValue0 );
//				
////				if( F4FlashMemorySize >0x400 )
////				{
////					writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR1);
////					writeAP(AP_DRW, protectValue1 );
////				}
//				
//				writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR );
//				writeAP(AP_DRW, protectValue0 + 2 );  // ��ʼ����
//				
//				do
//					{
//						writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//						readAP(AP_DRW,&readWord1);
//						readDP(DP_RDBUFF,&readWord1);
//					}while( (readWord1>>16)&0x1); // æʱ��ס
//			}
//	}

	u32 readWord1 = 0;
	
	u32 protectValue0 = 0; // �Ĵ���OPTCR��ֵ
//	u32 protectValue1 = 0; // �Ĵ���OPTCR1��ֵ
	
//	u32 F4FlashMemorySize = 0;  // �ڲ������������ͱ�������ʱ������Ҫ֪�����С����������û�ж����ֵ��
//		{ // ���Flash�Ĵ�С��
//			u32 apId = 0;
//			writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
//			writeAP( AP_CSW,0x2 );   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
//			writeAP( AP_TAR,(u32)(0x1FFF7A22));   // дĿ���ַ
//			readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
//			readDP( DP_RDBUFF, &apId );
//			F4FlashMemorySize = apId>>16;
//		}
		writeDP( DP_ABORT,0x1F);
	
	if( readProtectionEnable == ENABLE )  // ʹ�ܶ�����
	{
		if( writeProtectionEnable == ENABLE) // ʹ��д����
		{
			protectValue0 = 0x0000FFED - 1;  // ʹ��д������RDP��0xFF��ʹ��һ��������
//			protectValue1 = 0x00000000;  // ʹ��д����
		}
		else
		{
			protectValue0 = 0x0FFFFFED - 1;
//			protectValue1 = 0x0FFF0000;
		}
	}
	else
	{
		if( writeProtectionEnable == ENABLE)
		{
			protectValue0 = 0x0000AAED - 1;
//			protectValue1 = 0x00000000;
		}
		else  // ȫ������ʹ��
		{
			protectValue0 = 0x0FFFAAED - 1;
//			protectValue1 = 0x0FFF0000;
		}
	}
	
	{
			{  // ����Flash����  --- ���棺���ܽ���Flash���������ִ��󣬶���֮ǰ�Ѿ�����Flash���ˣ�����ʵ������ݣ���������ľ���ԭ���պ��ٽ�����������ǳ���������в���Flash�ĵط�����
//				writeAP( AP_CSW,0x23000002);   // 32λд����
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0x45670123 );
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0xCDEF89AB );
				
//				writeAP( AP_CSW,0x00000002);   // ��
//				u32 readWord0 = 0;
//				do
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//					readAP(AP_DRW,&readWord0);
//					readDP(DP_RDBUFF,&readWord0);
//				}while((readWord0>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
			}
			{  // ����ѡ���ֽ�Flash����
				writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
				writeAP(AP_DRW, 0x08192A3B);
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
				writeAP(AP_DRW, 0x4C5D6E7F);
			}
			{  // ����ǰҪȷ��û��Flash���в���
				writeAP( AP_CSW,0x00000002);   // ��
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord1);
					readDP(DP_RDBUFF,&readWord1);
				}while( (readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
			}	
			
			
			
			{
				writeAP( AP_CSW,0x23000002 ); // д
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
				writeAP(AP_DRW, protectValue0 );
				
//				if( F4FlashMemorySize >0x400 )
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR1);
//					writeAP(AP_DRW, protectValue1 );
//				}
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR );
				writeAP(AP_DRW, protectValue0 + 2 );  // ��ʼ����
				
				writeAP( AP_CSW,0x00000002);   // ��
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
//					readAP(AP_DRW,&readWord1);
//					readDP(DP_RDBUFF,&readWord1);
//				}
				do
					{
						writeAP(AP_TAR, (u32)&F4_FLASH->SR);
						readAP(AP_DRW,&readWord1);
						readDP(DP_RDBUFF,&readWord1);
					}while( (readWord1>>16)&0x1); // æʱ��ס
					
					{ // ��������������Ҫ����Ϊ����漰������������ѡ���ֽ����м���ʱ��ġ���ʱ����Ҫ�ϵ�����������ȽϹؼ�
						
						writeAP( AP_CSW,0x00000002);   // ��
						writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
						readAP(AP_DRW,&readWord1);
						readDP(DP_RDBUFF,&readWord1);
						
						if( ( readWord1 & 0xFFFFF00 ) == ( protectValue0 & 0xFFFFF00 ) )  // ��д��ֵ�Ͷ���ֵ���Ƚϣ�
						{
							return 0x55AA;
						}
						else
						{
							if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12+12*10,12,RED,"  ѡ���ֽڼ�����   " );
								}
								else
								{
									LCD_ShowString(0,12+12*10,12,RED,"  Option byte Err   " );
								}
							
							return 0x00;
						}
//						if( F4FlashMemorySize >0x400 )
//						{
//							writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR1);
//							readAP(AP_DRW,&readWord1);
//							readDP(DP_RDBUFF,&readWord1);
//							if( ( readWord1 & 0xFFF0000 ) == ( protectValue1 & 0xFFF0000 ) )  // ��д��ֵ�Ͷ���ֵ���Ƚϣ�
//							{
//								return 0x55AA;
//							}
//							else
//							{
//								LCD_ShowString(0,12+12*10,12,RED," OptionByteCheckFail!" );
//							}
//						}
						
					}
			}
	}
}
/**
 *  @B F4ϵ�еĶ�д��������
 *  ע�⣺F4ϵ�еı���������ֱ�Ӳ����Ĵ�����Ӧ�ÿ���32λ���ʵġ�
 *  
 */


u32 stm32f4Protection( u8 readProtectionEnable,u8 writeProtectionEnable )
{
	u32 readWord1 = 0;
	
	u32 protectValue0 = 0; // �Ĵ���OPTCR��ֵ
	u32 protectValue1 = 0; // �Ĵ���OPTCR1��ֵ
	
	u32 F4FlashMemorySize = 0;  // �ڲ������������ͱ�������ʱ������Ҫ֪�����С����������û�ж����ֵ��
		{ // ���Flash�Ĵ�С��
			u32 apId = 0;
			writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
			writeAP( AP_CSW,0x2 );   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
			writeAP( AP_TAR,(u32)(0x1FFF7A22));   // дĿ���ַ
			readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
			readDP( DP_RDBUFF, &apId );
			F4FlashMemorySize = apId>>16;
		}
		writeDP( DP_ABORT,0x1F);
	
	if( readProtectionEnable == ENABLE )  // ʹ�ܶ�����
	{
		if( writeProtectionEnable == ENABLE) // ʹ��д����
		{
			protectValue0 = 0x0000FFED - 1;  // ʹ��д������RDP��0xFF��ʹ��һ��������
			protectValue1 = 0x00000000;  // ʹ��д����
		}
		else
		{
			protectValue0 = 0x0FFFFFED - 1;
			protectValue1 = 0x0FFF0000;
		}
	}
	else
	{
		if( writeProtectionEnable == ENABLE)
		{
			protectValue0 = 0x0000AAED - 1;
			protectValue1 = 0x00000000;
		}
		else  // ȫ������ʹ��
		{
			protectValue0 = 0x0FFFAAED - 1;
			protectValue1 = 0x0FFF0000;
		}
	}
	
	{
			{  // ����Flash����  --- ���棺���ܽ���Flash���������ִ��󣬶���֮ǰ�Ѿ�����Flash���ˣ�����ʵ������ݣ���������ľ���ԭ���պ��ٽ�����������ǳ���������в���Flash�ĵط�����
//				writeAP( AP_CSW,0x23000002);   // 32λд����
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0x45670123 );
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0xCDEF89AB );
				
//				writeAP( AP_CSW,0x00000002);   // ��
//				u32 readWord0 = 0;
//				do
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//					readAP(AP_DRW,&readWord0);
//					readDP(DP_RDBUFF,&readWord0);
//				}while((readWord0>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
			}
			{  // ����ѡ���ֽ�Flash����
				writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
				writeAP(AP_DRW, 0x08192A3B);
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
				writeAP(AP_DRW, 0x4C5D6E7F);
			}
			{  // ����ǰҪȷ��û��Flash���в���
				writeAP( AP_CSW,0x00000002);   // ��
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord1);
					readDP(DP_RDBUFF,&readWord1);
				}while( (readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
			}	
			
			
			
			{
				writeAP( AP_CSW,0x23000002 ); // д
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
				writeAP(AP_DRW, protectValue0 );
				
				if( F4FlashMemorySize >0x400 )
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR1);
					writeAP(AP_DRW, protectValue1 );
				}
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR );
				writeAP(AP_DRW, protectValue0 + 2 );  // ��ʼ����
				
				writeAP( AP_CSW,0x00000002);   // ��
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
//					readAP(AP_DRW,&readWord1);
//					readDP(DP_RDBUFF,&readWord1);
//				}
				do
					{
						writeAP(AP_TAR, (u32)&F4_FLASH->SR);
						readAP(AP_DRW,&readWord1);
						readDP(DP_RDBUFF,&readWord1);
					}while( (readWord1>>16)&0x1); // æʱ��ס
					
					{ // ��������������Ҫ����Ϊ����漰������������ѡ���ֽ����м���ʱ��ġ���ʱ����Ҫ�ϵ�����������ȽϹؼ�
						
						writeAP( AP_CSW,0x00000002);   // ��
						writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
						readAP(AP_DRW,&readWord1);
						readDP(DP_RDBUFF,&readWord1);
						
						if( ( readWord1 & 0xFFFFF00 ) == ( protectValue0 & 0xFFFFF00 ) )  // ��д��ֵ�Ͷ���ֵ���Ƚϣ�
						{
							return 0x55AA;
						}
						else
						{
							if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12+12*10,12,RED,"  ѡ���ֽڼ�����   " );
								}
								else
								{
									LCD_ShowString(0,12+12*10,12,RED,"  Option byte Err   " );
								}
							return 0x00;
						}
						if( F4FlashMemorySize >0x400 )
						{
							writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR1);
							readAP(AP_DRW,&readWord1);
							readDP(DP_RDBUFF,&readWord1);
							if( ( readWord1 & 0xFFF0000 ) == ( protectValue1 & 0xFFF0000 ) )  // ��д��ֵ�Ͷ���ֵ���Ƚϣ�
							{
								return 0x55AA;
							}
							else
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12+12*10,12,RED,"  ѡ���ֽڼ�����   " );
								}
								else
								{
									LCD_ShowString(0,12+12*10,12,RED,"  Option byte Err   " );
								}
								return 0x00;
							}
						}
						
					}
			}
	}
}


/**
 *  @B F0ϵ�е����غ���
 *  
 */

u32 DownloadFlashOfSTM32F0xx( _FileInformation userfile )
{
	u32 returnNumber = 0;
	{
		{ // 
				{ // ��鱣������
							OS_ERR err;
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
							writeAP( AP_CSW,0x2 );   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
							writeAP( AP_TAR,(u32)&OB->RDP);   // дĿ���ַ
							readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
							readDP( DP_RDBUFF, &apId );
							if( apId == 0xFFFF55AA ) // ����ܲ�������һ������û�����ö�����
							{												// ����F0ϵ�У����ֶ�����ֵΪ��0xFFFF5AA5��Ҳ���ǣ�ʹ���˶�����������������ΪJlink��ʹ�õ�J-Flash�еļ��ܣ�
								{ // ���д����
									writeAP( AP_TAR,(u32)&OB->WRP0);
									readAP( AP_DRW,&apId);
									readDP( DP_RDBUFF, &apId );
									
									if( apId != 0xFFFFFFFF)  // ˵������д����
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      ����д����      ");
										}
										else
										{
											LCD_ShowString(0,12*10,12,RED, "  have write protect  ");
										}
										goto dealTmp0;
									}
									
									writeAP( AP_TAR,(u32)&OB->WRP2);
									readAP( AP_DRW,&apId);
									readDP( DP_RDBUFF, &apId );
									if( apId != 0xFFFFFFFF ) // ˵������д����
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      ���ֶ�����      ");
										}
										else
										{
											LCD_ShowString(0,12*10,12,RED, "      read protect    ");
										}
										goto dealTmp0;
									}
								}
							}
							else
							{ // �����˶�������Ҫ�����
												if( L_ENGLISH_VER != 0x55AA )
												{
													LCD_ShowString(0,12*10,12,RED, "      ���ֶ�����      ");
												}
												else
												{
													LCD_ShowString(0,12*10,12,RED, "      read protect    ");
												}
								dealTmp0:
												if( L_ENGLISH_VER != 0x55AA )
												{
														LCD_ShowString(0,12*11,12,RED, "    �����... 2S     ");
												}
												else
												{
													LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 2S    ");
												}
								{ // ���SWD�Ĵ���
									writeDP( DP_ABORT,0x1F);
									stm32f0Protection( DISABLE,DISABLE );
								}
								OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    �����... 1S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 1S    ");
								}
								OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    �����... 0S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 0S    ");
								}
								
								LCD_ShowString(0,12*10,12,RED, "  Protection Cleared ");
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    ��ϵ���������   ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "DownloadAgainWithoutPower");
								}
								{  // ��������ѡ�
										{
											writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
											writeAP(AP_TAR, (u32)&FLASH->CR.All);
											writeAP(AP_DRW, 0x00002080);
										 	OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
										}
										returnNumber = 0xFFBB;
										return returnNumber;
								}
								{
									returnNumber = 0x00;
								}
								return returnNumber;
							}
					}
			}
			
			{ // ���Flash
				if( userfile.ChipEraseWay == 00 )  // ȫ����
				{
					clearAllFlash(  );  // ��������ȫ��Flash�Ĳ���
				}
				else	// ���������ΪС�����������ʹ����������Ǵ�����Ҳ������1KB�����������������Լ򻯣������˷�ʱ�䣬��һ���汾����̫���⣡
							// ����֮�⣬���й������ڵĵ�ַ��
				{
					u32 startAddressTmp0 = 0x08000000; // ��ʼҳ���ڵ�ַ
					u32 endAddressTmp1 = 0x08000000;   // ����ҳ���ڵ�ַ
					u32 startRollingAddressTmp3 = 0x08000000;
					u32 endRollingAddressTmp4 = 0x08000000;
					for( u32 tmp5 = 0;tmp5<1024;tmp5++ ) // 1024�����Ϊ1M��С��
					{
						if( userfile.ProgramStartAddress < ( 0x08000000 + 0x400 * tmp5 ) )
						{
							startAddressTmp0 = 0x08000000 + 0x400 * (tmp5 - 1);
							break;
						}
					}
					for( u32 tmp6 = 0;tmp6<1024;tmp6++ ) // 
					{
						if( (userfile.ProgramStartAddress + userfile.ProgramSize) < ( 0x08000000 + 0x400 * tmp6 ) )
						{
							endAddressTmp1 = 0x08000000 + 0x400 * (tmp6 - 1);
							break;
						}
					}
					
					for( u32 tmp7 = 0;tmp7<1024;tmp7++ ) // 
					{
						if( userfile.RollingCodeStartAddress < ( 0x08000000 + 0x400 * tmp7 ) )
						{
							startRollingAddressTmp3 = 0x08000000 + 0x400 * (tmp7 - 1);
							break;
						}
					}
					for( u32 tmp8 = 0;tmp8<1024;tmp8++ ) // 
					{
						if( (userfile.RollingCodeStartAddress + 32 ) < ( 0x08000000 + 0x400 * tmp8 ) )  // �����SIZEֻ��32
						{
							endRollingAddressTmp4 = 0x08000000 + 0x400 * (tmp8 - 1);
							break;
						}
					}
					{ // ��SWD���Flash����
						writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
	
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY1);
						
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY2);
						
						writeAP( AP_CSW,0x00000002);   // ��
						u32 readWord0 = 0;
						do
						{
							writeAP(AP_TAR, (u32)&FLASH->SR.All);
							readAP(AP_DRW,&readWord0);
							readDP(DP_RDBUFF,&readWord0);
						}while(readWord0&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
						
						for( u32 addressTmp9 = startAddressTmp0; addressTmp9<= endAddressTmp1; addressTmp9 += 0x400 )  // ���������
						{
							writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000010 );  // ҳ����ʹ��
							
							writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
							writeAP( AP_DRW, addressTmp9 );  // д���ַ
							
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b01000010 );  // ��ʼ
							
							writeAP( AP_CSW,0x00000002);   // ��
							u32 readWord1 = 0;
							do
							{
								writeAP(AP_TAR, (u32)&FLASH->SR.All);
								readAP(AP_DRW,&readWord1);
								readDP(DP_RDBUFF,&readWord1);
							}while(readWord1&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
						}
						{ // �ָ��ֳ�
							writeAP( AP_CSW,0x23000002);  
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000000 );
						}
						if( userfile.RollingCodeFunction == 0x55 ) // ʹ�ܹ��������ֻ�е��������ʹ�ܵ�ʱ���ٲ�������ֹ�����˴���ĵ�ַ��
						{
							for( u32 addressTmp10 = startRollingAddressTmp3; addressTmp10<= endRollingAddressTmp4; addressTmp10 += 0x400 )  // ���������
							{
								writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000010 );  // ҳ����ʹ��
								
								writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
								writeAP( AP_DRW, addressTmp10 );  // д���ַ
								
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b01000010 );  // ��ʼ
								
								writeAP( AP_CSW,0x00000002);   // ��
								u32 readWord1 = 0;
								do
								{
									writeAP(AP_TAR, (u32)&FLASH->SR.All);
									readAP(AP_DRW,&readWord1);
									readDP(DP_RDBUFF,&readWord1);
								}while(readWord1&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
							}
							{ // �ָ��ֳ�
								writeAP( AP_CSW,0x23000002);  
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000000 );
							}
						}
						{ // ����Flash
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b10000000 );
						}
					}
				}
			}
			{ // дFlash��ͬʱҪд�����������
				
				{  // ����Flash����
					writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY1);
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY2);
				}
				{ // ʹ�ܱ༭λ
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, 0x1);  // ʹ�ܱ�̲�����
				}
				
				{ // д������
					u32 HowMany4K = 0;
					if( userfile.ProgramSize % 0x1000 != 0 )
						HowMany4K = userfile.ProgramSize/0x1000 + 1;
					else
						HowMany4K = userfile.ProgramSize/0x1000;
					
					for( u32 tmp11 = 0;tmp11<HowMany4K; tmp11++ )  // дFlash�Ĳ�����ÿ�β���4K�ֽ�
					{
						u32 Address = userfile.ProgramStartAddress + tmp11*0x1000;  // ��ʼ������ַ
						W25QXX_Read( W25Q4KBuf.databuf4K , userfile.ProgramSaveAtW25QAddress + tmp11*0x1000, 0x1000); // 4k��С��16�����£�����0x1000
						
						writeAP( AP_CSW,0x23000001 | 0x10 ); // ����Ϊ16bit,д���ҵ�ַ�Զ����ӣ�
						writeAP( AP_TAR, Address );  // ��ַ��
						for( u32 tmp12 = 0;tmp12 <0x1000; tmp12 += 2 ) // д��4K
						{
							{ // ʵ�⣬���԰�����ĸ�ʡ�Ե����������������ٶ�һ�������ǣ���56S - 36S����Ϊд�������ܸ�������Ҫ���������������ʱ�������պ��ֹ�ٶ�������Ӱ���ٶȣ�
//								writeAP( AP_CSW,0x00000002 );   // ��
//								do
//								{
//									writeAP(AP_TAR, (u32)&FLASH->SR.All);
//									readAP(AP_DRW,&readWord0);
//									readDP(DP_RDBUFF,&readWord0);
//								}while(readWord0&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
							}
							// ����ķ��������Ը�Ϊ4K�Զ����ӵģ�������Ҫ��ʱ�䣬������ʱ������������ٶȣ�Ҳ���Ǻ�����������ʱ�Ȳ����ǣ��պ�Ϊ�������ٶȣ��������������
							
							#define SWD_AUTO_ADDRESS_ADD_VALUE 0x400   // �ر𾯸棺STM32F0ϵ�еĵ�ַ�Զ����Ӵ�СΪ��1024����������F1��F4��4096����
							if( ((Address + tmp12) %SWD_AUTO_ADDRESS_ADD_VALUE) == 0 ) // ˵������4k�ٽ�㣬��Ҫ����д��ַ
							{
								writeAP( AP_TAR, Address + tmp12 );  // ��ַ��
							}
							
							if( (Address+tmp12)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 24) + ( W25Q4KBuf.databuf4K[tmp12] << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
							}
						}
						{ // ��������������������4K��С�������ݶ���10%Ϊ���ȣ�
							// ���ʹ��emWinӦ����һ���򵥵����⣡
							float tmp13 = (float)((float)((float)tmp11+1.0)/(float)HowMany4K);
							for( float tmp14 = 0.0;tmp14 <= (float)1.0; tmp14 += 0.1 )
							{
								if( tmp13 <= tmp14 ) // �ҵ���һ�ο�ʼС�ڵ��ڵ�ֵ��
								{
									if( (tmp14 > 0.00) && ( tmp14 <= 0.1) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >             10%");
										break;
									}
									else if( (tmp14 > 0.1) && (tmp14 <=0.2 ) )
									{
										LCD_ShowString(0,12*10,12, BLACK,"   >>            20%");
										break;
									}
									else if( (tmp14 > 0.2) && (tmp14 <=0.3) )
									{
										LCD_ShowString(0,12*10,12, BLACK,"   >>>           30%");
										break;
									}
									else if( (tmp14 > 0.3) && (tmp14 <=0.4) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>          40%");
										break;
									}
									else if( (tmp14 > 0.4) && (tmp14 <=0.5) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>         50%");
										break;
									}
									else if( (tmp14 > 0.5) && (tmp14 <=0.6) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>        60%");
										break;
									}
									else if( (tmp14 > 0.6) && (tmp14 <=0.7) )
									{
										LCD_ShowString(0,12*10,12, BLACK,"   >>>>>>>       70%");
										break;
									}
									else if( (tmp14 > 0.7) && (tmp14 <=0.8) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>      80%");
										break;
									}
									else if( (tmp14 > 0.8) && (tmp14 <=0.9) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>>     90%");
										break;
									}
//									else if( (tmp14 > 0.9) && (tmp14 <=2.1) )
//									{
//										LCD_ShowString(0,12*10,12, "   >>>>>>>>>>    100%");
//										break;
//									}
								}
							}
							if( tmp11 + 1 >= HowMany4K )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>>>   100%");
							}
						}
					}
				}
				{ // д������
					u32 programTimesTmp15 = 0;  // ʵ���Ѿ���̴���
					u32 valueTmp16 = 0; 				// ʵ��д��Ĺ���ֵ
					if( userfile.RollingCodeFunction == 0x55 ) // ��ʾ��Ҫ����������
					{
						
						if( userfile.DownloaderNumbers[255] != 0xFF )
						{
							programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + 256;
						}
						else
						{
							for( u16 Tmp16 = 0;Tmp16<256;Tmp16++ )
							{
								if( userfile.DownloaderNumbers[Tmp16] == 0xFF )
								{
									programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + Tmp16;  // ʵ�ʱ�̴���
									break;
								}
							}
						}
						valueTmp16 = userfile.RollingCodeStartValue + ( userfile.RollingCodeStepValue* programTimesTmp15 );
						writeAP( AP_TAR, userfile.RollingCodeStartAddress );  // ��ַ��
						{
							if( (userfile.RollingCodeStartAddress)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
							}
						}
						{
							writeAP( AP_TAR, userfile.RollingCodeStartAddress+2 );  // ��ַ��
							{
								if( (userfile.RollingCodeStartAddress+2)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
								}
								else
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
								}
							}
						}
					
						{ // ���������
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
							writeAP( AP_CSW,0x2 | 0x10);   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
							writeAP( AP_TAR,userfile.RollingCodeStartAddress);   // дĿ���ַ
							readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
							readDP( DP_RDBUFF, &apId );
							if( apId != valueTmp16 )
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    ����У��ʧ��     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "Roll code check failed");
								}
								returnNumber = 0x00;  // ������
								return returnNumber;
							}
						}
					}
				}
				{ // У�����������ʱֻʹ�ú�У�飡
					switch( userfile.FlashCheckWay )
					{
						case 0x00: // ������
							if( L_ENGLISH_VER != 0x55AA )
							{
								LCD_ShowString(0,12*11,12,RED, "     ��У��FLASH     ");
							}
							else
							{
								LCD_ShowString(0,12*11,12,RED, "   not check FLASH   ");
							}
							returnNumber = 0x55AA;
							break;
						case 0x01: // �������
							break;
						case 0x02: // CRCУ�飨��У�飩 -- �պ�Ӧ��ֻ��������У�鷽ʽ����Ϊ������Ҳû�б�Ҫ���ˣ�
							{ // ������У�鷽ʽ��������Ҫ����ʼ��ַ4�ֽڶ��룬�����С4�ֽڱ���������պ����Ҫ��֤�������֤���ˣ����Դ�����Ͻ��������⣡
								u16 sumTmp15 = 0;  // ���ĺ�У�����
								writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
								writeAP( AP_CSW,0x2 | 0x10);   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
								u32 addressTmp16 = userfile.ProgramStartAddress;
								writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
								
								u32 apId = 0;
								readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
								
								for( ;addressTmp16 < ( userfile.ProgramStartAddress + userfile.ProgramSize );addressTmp16 += 4 )
								{
//									u32 apId = 0;
//									if( ( addressTmp16 %0x1000 ) == 0 )
//									{
//										writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
//									}
//									readAP( AP_DRW,&apId);  // ���ڶ����ݵĻ���ÿ��ֻ�ܶ���һ�εġ�������һ����ʱ��
//									readDP( DP_RDBUFF, &apId );
//									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
									if( ( addressTmp16 %SWD_AUTO_ADDRESS_ADD_VALUE ) == 0 )
									{
										writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
										readAP( AP_DRW,&apId); 
									}
									readAP( AP_DRW,&apId);  // ����������ϴεġ�����Ҳ����Ҫ�ġ�
									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
								}
								if( sumTmp15 == userfile.SumValueOfProgram )
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12, BLACK,"   FLASH У��ɹ�   ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check success ");
									}
									returnNumber = 0x55AA;
								}
								else
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12,BLACK, "   FLASH У��ʧ��    ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check failure ");
									}
									returnNumber = 0x1122;
								}
							}
							break;
						case 0x03: // MD5ֵ����
							break;
						default:
							break;
					}
				}
				{ // д���룬��У�����������
					// Ϊ��ʹ��������ȽϷ��㣺Ҫ��1��������4�ֽڶ����ַ�ϣ�����ʹ��С��ģʽ��
					{ // ��������ַ
						
					}
					{ // д����
						
					}
					{ // У����룬ʹ��ÿ���Աȣ�
						
					}
				}
				{ // ������������(��ʵ����Ӧ�ü�һ����������ʹ�ܵĲ�����)
					switch( userfile.OptionBytesFunction )
					{
						case 0x55: // ʹ��ѡ���ֽڣ��պ��ܿ��ţ�
							break;
						case 0xAA: // ʹ�ܶ�����
							stm32f0Protection( ENABLE,DISABLE );
							break;
						case 0x11: // ʹ��д����
							stm32f0Protection( DISABLE,ENABLE );
							break;
						case 0x1A: // ʹ�ܶ�д����
							stm32f0Protection( ENABLE,ENABLE );
							break;
					}
				}
				{
					writeAP( AP_CSW,0x23000002); // д
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, _0b10000000);  // ��ֹ��̺�����
				}
				{
//					resetAndHaltTarget( );
					runTarget( );
				}
			}
	}
	return returnNumber;
}


u32 DownloadFlashOfSTM32F1xx( _FileInformation userfile )
{
	
	u32 returnNumber = 0;
	{ // 
				{ // ��鱣������
							OS_ERR err;
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
							writeAP( AP_CSW,0x2 );   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
							writeAP( AP_TAR,(u32)&OB->RDP);   // дĿ���ַ
							readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
							readDP( DP_RDBUFF, &apId );
							if( apId == 0xFFFF5AA5 ) // ����ܲ�������һ������û�����ö�����
							{
								{ // ���д����
									writeAP( AP_TAR,(u32)&OB->WRP0);
									readAP( AP_DRW,&apId);
									readDP( DP_RDBUFF, &apId );
									
									if( apId != 0xFFFFFFFF)  // ˵������д����
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      ����д����      ");
										}
										else
										{
											LCD_ShowString(0,12*10,12,RED, "  have write protect  ");
										}
										writeDP( DP_ABORT,0x1F);
										stm32f1Protection( ENABLE,DISABLE );
										return 0xFFBB;
//										goto dealTmp0;
									}
									
									writeAP( AP_TAR,(u32)&OB->WRP2);
									readAP( AP_DRW,&apId);
									readDP( DP_RDBUFF, &apId );
									if( apId != 0xFFFFFFFF ) // ˵������д����
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      ���ֶ�����      ");
										}
										else
										{
											LCD_ShowString(0,12*10,12,RED, "      read protect    ");
										}
										goto dealTmp0;
									}
								}
							}
							else
							{ // �����˶�������Ҫ�����
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*10,12,RED, "      ���ֶ�����      ");
								}
								else
								{
									LCD_ShowString(0,12*10,12,RED, "      read protect    ");
								}
								dealTmp0:
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    �����... 2S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 2S    ");
								}
								{ // ���SWD�Ĵ���
									writeDP( DP_ABORT,0x1F);
									
									stm32f1Protection( DISABLE,DISABLE );
								}
								OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    �����... 1S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 1S    ");
								}
								OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    �����... 0S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 0S    ");
								}
								
								LCD_ShowString(0,12*10,12,RED, "  Protection Cleared ");
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    ��ϵ���������   ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "Download again without power");
								}
								return 0xFFBB;
							}
					}
			}
			
			{ // ���Flash
				if( userfile.ChipEraseWay == 00 )  // ȫ����
				{
					clearAllFlash(  );  // ��������ȫ��Flash�Ĳ���
				}
				else	// ���������ΪС�����������ʹ����������Ǵ�����Ҳ������1KB�����������������Լ򻯣������˷�ʱ�䣬��һ���汾����̫���⣡
							// ����֮�⣬���й������ڵĵ�ַ��
				{
					u32 startAddressTmp0 = 0x08000000; // ��ʼҳ���ڵ�ַ
					u32 endAddressTmp1 = 0x08000000;   // ����ҳ���ڵ�ַ
					u32 startRollingAddressTmp3 = 0x08000000;
					u32 endRollingAddressTmp4 = 0x08000000;
					for( u32 tmp5 = 0;tmp5<1024;tmp5++ ) // 1024�����Ϊ1M��С��
					{
						if( userfile.ProgramStartAddress < ( 0x08000000 + 0x400 * tmp5 ) )
						{
							startAddressTmp0 = 0x08000000 + 0x400 * (tmp5 - 1);
							break;
						}
					}
					for( u32 tmp6 = 0;tmp6<1024;tmp6++ ) // 
					{
						if( (userfile.ProgramStartAddress + userfile.ProgramSize) < ( 0x08000000 + 0x400 * tmp6 ) )
						{
							endAddressTmp1 = 0x08000000 + 0x400 * (tmp6 - 1);
							break;
						}
					}
					
					for( u32 tmp7 = 0;tmp7<1024;tmp7++ ) // 
					{
						if( userfile.RollingCodeStartAddress < ( 0x08000000 + 0x400 * tmp7 ) )
						{
							startRollingAddressTmp3 = 0x08000000 + 0x400 * (tmp7 - 1);
							break;
						}
					}
					for( u32 tmp8 = 0;tmp8<1024;tmp8++ ) // 
					{
						if( (userfile.RollingCodeStartAddress + 32 ) < ( 0x08000000 + 0x400 * tmp8 ) )  // �����SIZEֻ��32
						{
							endRollingAddressTmp4 = 0x08000000 + 0x400 * (tmp8 - 1);
							break;
						}
					}
					{ // ��SWD���Flash����
						writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
	
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY1);
						
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY2);
						
						writeAP( AP_CSW,0x00000002);   // ��
						u32 readWord0 = 0;
						do
						{
							writeAP(AP_TAR, (u32)&FLASH->SR.All);
							readAP(AP_DRW,&readWord0);
							readDP(DP_RDBUFF,&readWord0);
						}while(readWord0&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
						
						for( u32 addressTmp9 = startAddressTmp0; addressTmp9<= endAddressTmp1; addressTmp9 += 0x400 )  // ���������
						{
							writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000010 );  // ҳ����ʹ��
							
							writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
							writeAP( AP_DRW, addressTmp9 );  // д���ַ
							
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b01000010 );  // ��ʼ
							
							writeAP( AP_CSW,0x00000002);   // ��
							u32 readWord1 = 0;
							do
							{
								writeAP(AP_TAR, (u32)&FLASH->SR.All);
								readAP(AP_DRW,&readWord1);
								readDP(DP_RDBUFF,&readWord1);
							}while(readWord1&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
						}
						{ // �ָ��ֳ�
							writeAP( AP_CSW,0x23000002);  
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000000 );
						}
						if( userfile.RollingCodeFunction == 0x55 ) // ʹ�ܹ��������ֻ�е��������ʹ�ܵ�ʱ���ٲ�������ֹ�����˴���ĵ�ַ��
						{
							for( u32 addressTmp10 = startRollingAddressTmp3; addressTmp10<= endRollingAddressTmp4; addressTmp10 += 0x400 )  // ���������
							{
								writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000010 );  // ҳ����ʹ��
								
								writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
								writeAP( AP_DRW, addressTmp10 );  // д���ַ
								
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b01000010 );  // ��ʼ
								
								writeAP( AP_CSW,0x00000002);   // ��
								u32 readWord1 = 0;
								do
								{
									writeAP(AP_TAR, (u32)&FLASH->SR.All);
									readAP(AP_DRW,&readWord1);
									readDP(DP_RDBUFF,&readWord1);
								}while(readWord1&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
							}
							{ // �ָ��ֳ�
								writeAP( AP_CSW,0x23000002);  
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000000 );
							}
						}
						{ // ����Flash
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b10000000 );
						}
					}
				}
			}
			{ // дFlash��ͬʱҪд�����������
				
				{  // ����Flash����
					writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY1);
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY2);
				}
				{ // ʹ�ܱ༭λ
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, 0x1);  // ʹ�ܱ�̲�����
				}
				
				{ // д������
					u32 HowMany4K = 0;
					if( userfile.ProgramSize % 0x1000 != 0 )
						HowMany4K = userfile.ProgramSize/0x1000 + 1;  // ֮ǰ��ȫ��д�ɣ�4000��ʵ��Ӧ���ǣ�
					else
						HowMany4K = userfile.ProgramSize/0x1000;
					
					for( u32 tmp11 = 0;tmp11<HowMany4K; tmp11++ )  // дFlash�Ĳ�����ÿ�β���4K�ֽ�
					{
						u32 Address = userfile.ProgramStartAddress + tmp11*0x1000;  // ��ʼ������ַ
						W25QXX_Read( W25Q4KBuf.databuf4K , userfile.ProgramSaveAtW25QAddress + tmp11*0x1000, 0x1000); // 4k��С��16�����£�����0x1000
						
						writeAP( AP_CSW,0x23000001 | 0x10 ); // ����Ϊ16bit,д���ҵ�ַ�Զ����ӣ�
						writeAP( AP_TAR, Address );  // ��ַ��
						for( u32 tmp12 = 0;tmp12 <0x1000; tmp12 += 2 ) // д��4K
						{
							{ // ʵ�⣬���԰�����ĸ�ʡ�Ե����������������ٶ�һ�������ǣ���56S - 36S����Ϊд�������ܸ�������Ҫ���������������ʱ�������պ��ֹ�ٶ�������Ӱ���ٶȣ�
//								writeAP( AP_CSW,0x00000002 );   // ��
//								do
//								{
//									writeAP(AP_TAR, (u32)&FLASH->SR.All);
//									readAP(AP_DRW,&readWord0);
//									readDP(DP_RDBUFF,&readWord0);
//								}while(readWord0&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
							}
							// ����ķ��������Ը�Ϊ4K�Զ����ӵģ�������Ҫ��ʱ�䣬������ʱ������������ٶȣ�Ҳ���Ǻ�����������ʱ�Ȳ����ǣ��պ�Ϊ�������ٶȣ��������������
							if( ((Address + tmp12) %0x1000) == 0 ) // ˵������4k�ٽ�㣬��Ҫ����д��ַ
							{
								writeAP( AP_TAR, Address + tmp12 );  // ��ַ��
							}
							
							
							
							
							
							if( (Address+tmp12)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 24) + ( W25Q4KBuf.databuf4K[tmp12] << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
							}
						}
						{ // ��������������������4K��С�������ݶ���10%Ϊ���ȣ�
							// ���ʹ��emWinӦ����һ���򵥵����⣡
							float tmp13 = (float)((float)((float)tmp11+1.0)/(float)HowMany4K);
							for( float tmp14 = 0.0;tmp14 <= (float)1.0; tmp14 += 0.1 )
							{
								if( tmp13 <= tmp14 ) // �ҵ���һ�ο�ʼС�ڵ��ڵ�ֵ��
								{
									if( (tmp14 > 0.00) && ( tmp14 <= 0.1) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >             10%");
										break;
									}
									else if( (tmp14 > 0.1) && (tmp14 <=0.2 ) )
									{
										LCD_ShowString(0,12*10,12, BLACK,"   >>            20%");
										break;
									}
									else if( (tmp14 > 0.2) && (tmp14 <=0.3) )
									{
										LCD_ShowString(0,12*10,12, BLACK,"   >>>           30%");
										break;
									}
									else if( (tmp14 > 0.3) && (tmp14 <=0.4) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>          40%");
										break;
									}
									else if( (tmp14 > 0.4) && (tmp14 <=0.5) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>         50%");
										break;
									}
									else if( (tmp14 > 0.5) && (tmp14 <=0.6) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>        60%");
										break;
									}
									else if( (tmp14 > 0.6) && (tmp14 <=0.7) )
									{
										LCD_ShowString(0,12*10,12, BLACK,"   >>>>>>>       70%");
										break;
									}
									else if( (tmp14 > 0.7) && (tmp14 <=0.8) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>      80%");
										break;
									}
									else if( (tmp14 > 0.8) && (tmp14 <=0.9) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>>     90%");
										break;
									}
//									else if( (tmp14 > 0.9) && (tmp14 <=2.1) )
//									{
//										LCD_ShowString(0,12*10,12, "   >>>>>>>>>>    100%");
//										break;
//									}
								}
							}
							if( tmp11 + 1 >= HowMany4K )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>>>   100%");
							}
						}
					}
				}
				{ // д������
					u32 programTimesTmp15 = 0;  // ʵ���Ѿ���̴���
					u32 valueTmp16 = 0; 				// ʵ��д��Ĺ���ֵ
					if( userfile.RollingCodeFunction == 0x55 ) // ��ʾ��Ҫ����������
					{
						
						if( userfile.DownloaderNumbers[255] != 0xFF )
						{
							programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + 256;
						}
						else
						{
							for( u16 Tmp16 = 0;Tmp16<256;Tmp16++ )
							{
								if( userfile.DownloaderNumbers[Tmp16] == 0xFF )
								{
									programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + Tmp16;  // ʵ�ʱ�̴���
									break;
								}
							}
						}
						valueTmp16 = userfile.RollingCodeStartValue + ( userfile.RollingCodeStepValue* programTimesTmp15 );
						writeAP( AP_TAR, userfile.RollingCodeStartAddress );  // ��ַ��
						{
							if( (userfile.RollingCodeStartAddress)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
							}
						}
						{
							writeAP( AP_TAR, userfile.RollingCodeStartAddress+2 );  // ��ַ��
							{
								if( (userfile.RollingCodeStartAddress+2)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
								}
								else
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
								}
							}
						}
					
						{ // ���������
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
							writeAP( AP_CSW,0x2 | 0x10);   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
							writeAP( AP_TAR,userfile.RollingCodeStartAddress);   // дĿ���ַ
							readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
							readDP( DP_RDBUFF, &apId );
							if( apId != valueTmp16 )
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    ����У��ʧ��     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "Roll code check failed");
								}
								returnNumber = 0x00;  // ������
								return returnNumber;
							}
						}
					}
				}
				{ // У�����������ʱֻʹ�ú�У�飡
					switch( userfile.FlashCheckWay )
					{
						case 0x00: // ������
							if( L_ENGLISH_VER != 0x55AA )
							{
								LCD_ShowString(0,12*11,12,RED, "     ��У��FLASH     ");
							}
							else
							{
								LCD_ShowString(0,12*11,12,RED, "   not check FLASH   ");
							}
							returnNumber = 0x55AA;
							break;
						case 0x01: // �������
							break;
						case 0x02: // CRCУ�飨��У�飩 -- �պ�Ӧ��ֻ��������У�鷽ʽ����Ϊ������Ҳû�б�Ҫ���ˣ�
							{ // ������У�鷽ʽ��������Ҫ����ʼ��ַ4�ֽڶ��룬�����С4�ֽڱ���������պ����Ҫ��֤�������֤���ˣ����Դ�����Ͻ��������⣡
								u16 sumTmp15 = 0;  // ���ĺ�У�����
								writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
								writeAP( AP_CSW,0x2 | 0x10);   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
								u32 addressTmp16 = userfile.ProgramStartAddress;
								writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
								
								u32 apId = 0;
								readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
								
								for( ;addressTmp16 < ( userfile.ProgramStartAddress + userfile.ProgramSize );addressTmp16 += 4 )
								{
//									u32 apId = 0;
//									if( ( addressTmp16 %0x1000 ) == 0 )
//									{
//										writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
//									}
//									readAP( AP_DRW,&apId);  // ���ڶ����ݵĻ���ÿ��ֻ�ܶ���һ�εġ�������һ����ʱ��
//									readDP( DP_RDBUFF, &apId );
//									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
									if( ( addressTmp16 %0x1000 ) == 0 )
									{
										writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
										readAP( AP_DRW,&apId); 
									}
									readAP( AP_DRW,&apId);  // ����������ϴεġ�����Ҳ����Ҫ�ġ�
									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
								}
								if( sumTmp15 == userfile.SumValueOfProgram )
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12, BLACK,"   FLASH У��ɹ�   ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check success ");
									}
									returnNumber = 0x55AA;
								}
								else
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12,BLACK, "   FLASH У��ʧ��    ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check failure ");
									}
									returnNumber = 0x1122;
									return returnNumber;  // �����ʱ��������з��أ���Ϊ�����������ʹ�ܱ���������ϵ��ʱ����Ҫ�ϵ��������أ�����GD32������ʱ�������⣡
								}
							}
							break;
						case 0x03: // MD5ֵ����
							break;
						default:
							break;
					}
				}
				{ // д���룬��У�����������
					// Ϊ��ʹ��������ȽϷ��㣺Ҫ��1��������4�ֽڶ����ַ�ϣ�����ʹ��С��ģʽ��
					{ // ��������ַ
						
					}
					{ // д����
						
					}
					{ // У����룬ʹ��ÿ���Աȣ�
						
					}
				}
				{ // ������������(��ʵ����Ӧ�ü�һ����������ʹ�ܵĲ�����)
					switch( userfile.OptionBytesFunction )
					{
						case 0x55: // ʹ��ѡ���ֽڣ��պ��ܿ��ţ�
							break;
						case 0xAA: // ʹ�ܶ�����
							stm32f1Protection( ENABLE,DISABLE );
							break;
						case 0x11: // ʹ��д����
							stm32f1Protection( DISABLE,ENABLE );
							break;
						case 0x1A: // ʹ�ܶ�д����
							stm32f1Protection( ENABLE,ENABLE );
							break;
					}
				}
				{
					writeAP( AP_CSW,0x23000002); // д
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, _0b10000000);  // ��ֹ��̺�����
				}
				{
					resetAndHaltTarget( );
					runTarget( );
				}
			}
			
			return returnNumber;
}

// F3ϵ�е�֧�֣�2018.07.09 - ��ʱû��ͨ������

u32 DownloadFlashOfSTM32F3xx( _FileInformation userfile )
{
	u32 returnNumber = 0;
	{ // 
				{ // ��鱣������
							OS_ERR err;
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
							writeAP( AP_CSW,0x2 );   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
							writeAP( AP_TAR,(u32)&OB->RDP);   // дĿ���ַ
							readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
							readDP( DP_RDBUFF, &apId );
							if( (apId & 0xFFFF) == 0x55AA ) // ����ܲ�������һ������û�����ö�����
							{
								{ // ���д����
									writeAP( AP_TAR,(u32)&OB->WRP0);
									readAP( AP_DRW,&apId);
									readDP( DP_RDBUFF, &apId );
									
									if( apId != 0xFFFFFFFF)  // ˵������д����
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      ����д����      ");
										}
										else
										{
											LCD_ShowString(0,12*10,12,RED, "  have write protect  ");
										}
										goto dealTmp0;
									}
									
									writeAP( AP_TAR,(u32)&OB->WRP2);
									readAP( AP_DRW,&apId);
									readDP( DP_RDBUFF, &apId );
									if( apId != 0xFFFFFFFF ) // ˵������д����
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      ���ֶ�����      ");
										}
										else
										{
											LCD_ShowString(0,12*10,12,RED, "      read protect    ");
										}
										goto dealTmp0;
									}
								}
							}
							else
							{ // �����˶�������Ҫ�����
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*10,12,RED, "      ���ֶ�����      ");
								}
								else
								{
									LCD_ShowString(0,12*10,12,RED, "      read protect    ");
								}
								dealTmp0:
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    �����... 2S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 2S    ");
								}
								{ // ���SWD�Ĵ���
									writeDP( DP_ABORT,0x1F);
									stm32f3Protection( DISABLE,DISABLE );
								}
								OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    �����... 2S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 2S    ");
								}
								OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    �����... 2S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 2S    ");
								}
								
								LCD_ShowString(0,12*10,12,RED, "  Protection Cleared ");
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    ��ϵ���������   ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "Download again without power");
								}
								return 0x00;
							}
					}
			}
			
			{ // ���Flash
				if( userfile.ChipEraseWay == 00 )  // ȫ����
				{
					clearAllFlash(  );  // ��������ȫ��Flash�Ĳ���
				}
				else	// ���������ΪС�����������ʹ����������Ǵ�����Ҳ������1KB�����������������Լ򻯣������˷�ʱ�䣬��һ���汾����̫���⣡
							// ����֮�⣬���й������ڵĵ�ַ��
				{
					u32 startAddressTmp0 = 0x08000000; // ��ʼҳ���ڵ�ַ
					u32 endAddressTmp1 = 0x08000000;   // ����ҳ���ڵ�ַ
					u32 startRollingAddressTmp3 = 0x08000000;
					u32 endRollingAddressTmp4 = 0x08000000;
					for( u32 tmp5 = 0;tmp5<1024;tmp5++ ) // 1024�����Ϊ1M��С��
					{
						if( userfile.ProgramStartAddress < ( 0x08000000 + 0x400 * tmp5 ) )
						{
							startAddressTmp0 = 0x08000000 + 0x400 * (tmp5 - 1);
							break;
						}
					}
					for( u32 tmp6 = 0;tmp6<1024;tmp6++ ) // 
					{
						if( (userfile.ProgramStartAddress + userfile.ProgramSize) < ( 0x08000000 + 0x400 * tmp6 ) )
						{
							endAddressTmp1 = 0x08000000 + 0x400 * (tmp6 - 1);
							break;
						}
					}
					
					for( u32 tmp7 = 0;tmp7<1024;tmp7++ ) // 
					{
						if( userfile.RollingCodeStartAddress < ( 0x08000000 + 0x400 * tmp7 ) )
						{
							startRollingAddressTmp3 = 0x08000000 + 0x400 * (tmp7 - 1);
							break;
						}
					}
					for( u32 tmp8 = 0;tmp8<1024;tmp8++ ) // 
					{
						if( (userfile.RollingCodeStartAddress + 32 ) < ( 0x08000000 + 0x400 * tmp8 ) )  // �����SIZEֻ��32
						{
							endRollingAddressTmp4 = 0x08000000 + 0x400 * (tmp8 - 1);
							break;
						}
					}
					{ // ��SWD���Flash����
						writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
	
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY1);
						
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY2);
						
						writeAP( AP_CSW,0x00000002);   // ��
						u32 readWord0 = 0;
						do
						{
							writeAP(AP_TAR, (u32)&FLASH->SR.All);
							readAP(AP_DRW,&readWord0);
							readDP(DP_RDBUFF,&readWord0);
						}while(readWord0&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
						
						for( u32 addressTmp9 = startAddressTmp0; addressTmp9<= endAddressTmp1; addressTmp9 += 0x400 )  // ���������
						{
							writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000010 );  // ҳ����ʹ��
							
							writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
							writeAP( AP_DRW, addressTmp9 );  // д���ַ
							
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b01000010 );  // ��ʼ
							
							writeAP( AP_CSW,0x00000002);   // ��
							u32 readWord1 = 0;
							do
							{
								writeAP(AP_TAR, (u32)&FLASH->SR.All);
								readAP(AP_DRW,&readWord1);
								readDP(DP_RDBUFF,&readWord1);
							}while(readWord1&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
						}
						{ // �ָ��ֳ�
							writeAP( AP_CSW,0x23000002);  
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000000 );
						}
						if( userfile.RollingCodeFunction == 0x55 ) // ʹ�ܹ��������ֻ�е��������ʹ�ܵ�ʱ���ٲ�������ֹ�����˴���ĵ�ַ��
						{
							for( u32 addressTmp10 = startRollingAddressTmp3; addressTmp10<= endRollingAddressTmp4; addressTmp10 += 0x400 )  // ���������
							{
								writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000010 );  // ҳ����ʹ��
								
								writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
								writeAP( AP_DRW, addressTmp10 );  // д���ַ
								
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b01000010 );  // ��ʼ
								
								writeAP( AP_CSW,0x00000002);   // ��
								u32 readWord1 = 0;
								do
								{
									writeAP(AP_TAR, (u32)&FLASH->SR.All);
									readAP(AP_DRW,&readWord1);
									readDP(DP_RDBUFF,&readWord1);
								}while(readWord1&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
							}
							{ // �ָ��ֳ�
								writeAP( AP_CSW,0x23000002);  
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000000 );
							}
						}
						{ // ����Flash
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b10000000 );
						}
					}
				}
			}
			{ // дFlash��ͬʱҪд�����������
				
				{  // ����Flash����
					writeAP( AP_CSW,0x23000002);   // һ��Ҫ����ǰд���������������ʾ��32λ��ͬʱ�������ʵķ�����д���ʣ�����ᱨ����
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY1);
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY2);
				}
				{ // ʹ�ܱ༭λ
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, 0x1);  // ʹ�ܱ�̲�����
				}
				
				{ // д������
					u32 HowMany4K = 0;
					if( userfile.ProgramSize % 0x1000 != 0 )
						HowMany4K = userfile.ProgramSize/0x1000 + 1;
					else
						HowMany4K = userfile.ProgramSize/0x1000;
					
					for( u32 tmp11 = 0;tmp11<HowMany4K; tmp11++ )  // дFlash�Ĳ�����ÿ�β���4K�ֽ�
					{
						u32 Address = userfile.ProgramStartAddress + tmp11*0x1000;  // ��ʼ������ַ
						W25QXX_Read( W25Q4KBuf.databuf4K , userfile.ProgramSaveAtW25QAddress + tmp11*0x1000, 0x1000); // 4k��С��16�����£�����0x1000
						
						writeAP( AP_CSW,0x23000001 | 0x10 ); // ����Ϊ16bit,д���ҵ�ַ�Զ����ӣ�
						writeAP( AP_TAR, Address );  // ��ַ��
						for( u32 tmp12 = 0;tmp12 <0x1000; tmp12 += 2 ) // д��4K
						{
							{ // ʵ�⣬���԰�����ĸ�ʡ�Ե����������������ٶ�һ�������ǣ���56S - 36S����Ϊд�������ܸ�������Ҫ���������������ʱ�������պ��ֹ�ٶ�������Ӱ���ٶȣ�
//								writeAP( AP_CSW,0x00000002 );   // ��
//								do
//								{
//									writeAP(AP_TAR, (u32)&FLASH->SR.All);
//									readAP(AP_DRW,&readWord0);
//									readDP(DP_RDBUFF,&readWord0);
//								}while(readWord0&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
							}
							// ����ķ��������Ը�Ϊ4K�Զ����ӵģ�������Ҫ��ʱ�䣬������ʱ������������ٶȣ�Ҳ���Ǻ�����������ʱ�Ȳ����ǣ��պ�Ϊ�������ٶȣ��������������
							if( ((Address + tmp12) %0x1000) == 0 ) // ˵������4k�ٽ�㣬��Ҫ����д��ַ
							{
								writeAP( AP_TAR, Address + tmp12 );  // ��ַ��
							}
							
							if( (Address+tmp12)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 24) + ( W25Q4KBuf.databuf4K[tmp12] << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
							}
						}
						{ // ��������������������4K��С�������ݶ���10%Ϊ���ȣ�
							// ���ʹ��emWinӦ����һ���򵥵����⣡
							float tmp13 = (float)((float)((float)tmp11+1.0)/(float)HowMany4K);
							for( float tmp14 = 0.0;tmp14 <= (float)1.0; tmp14 += 0.1 )
							{
								if( tmp13 <= tmp14 ) // �ҵ���һ�ο�ʼС�ڵ��ڵ�ֵ��
								{
									if( (tmp14 > 0.00) && ( tmp14 <= 0.1) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >             10%");
										break;
									}
									else if( (tmp14 > 0.1) && (tmp14 <=0.2 ) )
									{
										LCD_ShowString(0,12*10,12, BLACK,"   >>            20%");
										break;
									}
									else if( (tmp14 > 0.2) && (tmp14 <=0.3) )
									{
										LCD_ShowString(0,12*10,12, BLACK,"   >>>           30%");
										break;
									}
									else if( (tmp14 > 0.3) && (tmp14 <=0.4) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>          40%");
										break;
									}
									else if( (tmp14 > 0.4) && (tmp14 <=0.5) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>         50%");
										break;
									}
									else if( (tmp14 > 0.5) && (tmp14 <=0.6) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>        60%");
										break;
									}
									else if( (tmp14 > 0.6) && (tmp14 <=0.7) )
									{
										LCD_ShowString(0,12*10,12, BLACK,"   >>>>>>>       70%");
										break;
									}
									else if( (tmp14 > 0.7) && (tmp14 <=0.8) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>      80%");
										break;
									}
									else if( (tmp14 > 0.8) && (tmp14 <=0.9) )
									{
										LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>>     90%");
										break;
									}
//									else if( (tmp14 > 0.9) && (tmp14 <=2.1) )
//									{
//										LCD_ShowString(0,12*10,12, "   >>>>>>>>>>    100%");
//										break;
//									}
								}
							}
							if( tmp11 + 1 >= HowMany4K )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>>>   100%");
							}
						}
					}
				}
				{ // д������
					u32 programTimesTmp15 = 0;  // ʵ���Ѿ���̴���
					u32 valueTmp16 = 0; 				// ʵ��д��Ĺ���ֵ
					if( userfile.RollingCodeFunction == 0x55 ) // ��ʾ��Ҫ����������
					{
						
						if( userfile.DownloaderNumbers[255] != 0xFF )
						{
							programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + 256;
						}
						else
						{
							for( u16 Tmp16 = 0;Tmp16<256;Tmp16++ )
							{
								if( userfile.DownloaderNumbers[Tmp16] == 0xFF )
								{
									programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + Tmp16;  // ʵ�ʱ�̴���
									break;
								}
							}
						}
						valueTmp16 = userfile.RollingCodeStartValue + ( userfile.RollingCodeStepValue* programTimesTmp15 );
						writeAP( AP_TAR, userfile.RollingCodeStartAddress );  // ��ַ��
						{
							if( (userfile.RollingCodeStartAddress)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
							}
						}
						{
							writeAP( AP_TAR, userfile.RollingCodeStartAddress+2 );  // ��ַ��
							{
								if( (userfile.RollingCodeStartAddress+2)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
								}
								else
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
								}
							}
						}
					
						{ // ���������
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
							writeAP( AP_CSW,0x2 | 0x10);   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
							writeAP( AP_TAR,userfile.RollingCodeStartAddress);   // дĿ���ַ
							readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
							readDP( DP_RDBUFF, &apId );
							if( apId != valueTmp16 )
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    ����У��ʧ��     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "Roll code check failed");
								}
								returnNumber = 0x00;  // ������
								return returnNumber;
							}
						}
					}
				}
				{ // У�����������ʱֻʹ�ú�У�飡
					switch( userfile.FlashCheckWay )
					{
						case 0x00: // ������
							if( L_ENGLISH_VER != 0x55AA )
							{
								LCD_ShowString(0,12*11,12,RED, "     ��У��FLASH     ");
							}
							else
							{
								LCD_ShowString(0,12*11,12,RED, "   not check FLASH   ");
							}
							returnNumber = 0x55AA;
							break;
						case 0x01: // �������
							break;
						case 0x02: // CRCУ�飨��У�飩 -- �պ�Ӧ��ֻ��������У�鷽ʽ����Ϊ������Ҳû�б�Ҫ���ˣ�
							{ // ������У�鷽ʽ��������Ҫ����ʼ��ַ4�ֽڶ��룬�����С4�ֽڱ���������պ����Ҫ��֤�������֤���ˣ����Դ�����Ͻ��������⣡
								u16 sumTmp15 = 0;  // ���ĺ�У�����
								writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
								writeAP( AP_CSW,0x2 | 0x10);   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
								u32 addressTmp16 = userfile.ProgramStartAddress;
								writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
								
								u32 apId = 0;
								readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
								
								for( ;addressTmp16 < ( userfile.ProgramStartAddress + userfile.ProgramSize );addressTmp16 += 4 )
								{
//									u32 apId = 0;
//									if( ( addressTmp16 %0x1000 ) == 0 )
//									{
//										writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
//									}
//									readAP( AP_DRW,&apId);  // ���ڶ����ݵĻ���ÿ��ֻ�ܶ���һ�εġ�������һ����ʱ��
//									readDP( DP_RDBUFF, &apId );
//									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
									if( ( addressTmp16 %0x1000 ) == 0 )
									{
										writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
										readAP( AP_DRW,&apId); 
									}
									readAP( AP_DRW,&apId);  // ����������ϴεġ�����Ҳ����Ҫ�ġ�
									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
								}
								if( sumTmp15 == userfile.SumValueOfProgram )
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12, BLACK,"   FLASH У��ɹ�   ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check success ");
									}
									returnNumber = 0x55AA;
								}
								else
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12,BLACK, "   FLASH У��ʧ��    ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check failure ");
									}
									returnNumber = 0x1122;
								}
							}
							break;
						case 0x03: // MD5ֵ����
							break;
						default:
							break;
					}
				}
				{ // д���룬��У�����������
					// Ϊ��ʹ��������ȽϷ��㣺Ҫ��1��������4�ֽڶ����ַ�ϣ�����ʹ��С��ģʽ��
					{ // ��������ַ
						
					}
					{ // д����
						
					}
					{ // У����룬ʹ��ÿ���Աȣ�
						
					}
				}
				{ // ������������(��ʵ����Ӧ�ü�һ����������ʹ�ܵĲ�����)
					switch( userfile.OptionBytesFunction )
					{
						case 0x55: // ʹ��ѡ���ֽڣ��պ��ܿ��ţ�
							break;
						case 0xAA: // ʹ�ܶ�����
							stm32f1Protection( ENABLE,DISABLE );
							break;
						case 0x11: // ʹ��д����
							stm32f1Protection( DISABLE,ENABLE );
							break;
						case 0x1A: // ʹ�ܶ�д����
							stm32f1Protection( ENABLE,ENABLE );
							break;
					}
				}
				{
					writeAP( AP_CSW,0x23000002); // д
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, _0b10000000);  // ��ֹ��̺�����
				}
				{
					resetAndHaltTarget( );
					runTarget( );
				}
			}
			
			return returnNumber;
}

/**
 *  @B F2ϵ�е����غ��������ݶԱ��ֲᣬ����F2��F4�ǳ������ƣ�������ʱʹ��F4�ĺ������ĸģ�
 *  
 */

u32 DownloadFlashOfSTM32F2xx( _FileInformation userfile )
{
	u32 returnNumber = 0;
//	u32 F4FlashMemorySize = 0;  // �ڲ������������ͱ�������ʱ������Ҫ֪�����С����������û�ж����ֵ��
	{ // 
//		{ // ���Flash�Ĵ�С��
//			u32 apId = 0;
//			writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
//			writeAP( AP_CSW,0x2 );   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
//			writeAP( AP_TAR,(u32)(0x1FFF7A22));   // дĿ���ַ
//			readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
//			readDP( DP_RDBUFF, &apId );
//			F4FlashMemorySize = apId>>16;
//		}
		{ // ��鱣������
					OS_ERR err;
					u32 apId = 0;
					writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
					writeAP( AP_CSW,0x2 );   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
					writeAP( AP_TAR,(u32)&F4_FLASH->OPTCR);   // дĿ���ַ
					readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
					readDP( DP_RDBUFF, &apId );
			
			
					if( ((apId>>8)&(0xFF)) == 0xAA ) // û�м��������
					{
						{ // ���д����
							if( ((apId>>16)&(0x0FFF)) != 0x0FFF )  // ˵������д����
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*10,12,RED, "      ����д����      ");
								}
								else
								{
									LCD_ShowString(0,12*10,12,RED, "  have write protect  ");
								}
								goto dealTmp0;
							}
							
//							if( F4FlashMemorySize > 0x400 ) // ����1M�Ĵ洢
//							{
//								writeAP( AP_TAR,(u32)&F4_FLASH->OPTCR1);
//								readAP( AP_DRW,&apId);
//								readDP( DP_RDBUFF, &apId );
//								if( ((apId>>16)&(0x0FFF)) != 0x0FFF )  // ˵������д����
//								{
//									LCD_ShowString(0,12*10,12,RED, "   WRITE protection  ");
//									goto dealTmp0;
//								}
//							}
						}
					}
					else if( ((apId>>8)&(0xFF)) == 0xCC )  // �����Իָ��Ķ�����
					{
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*10,12,RED, "         ����        ");
							LCD_ShowString(0,12*11,12,RED, "       ������-LV2    ");
						}
						else
						{
							LCD_ShowString(0,12*10,12,RED, "       warning     ");
							LCD_ShowString(0,12*11,12,RED, "    ReadProtect-LV2  ");
						}
						return 0x00;
					}
					else  // �����˿�������Ķ�����
					{ // �����˶�������Ҫ�����
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*10,12,RED, "      ���ֶ�����      ");
						}
						else
						{
							LCD_ShowString(0,12*10,12,RED, "      read protect    ");
						}
						dealTmp0:
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*11,12,RED, "    �����...      ");
						}
						else
						{
							LCD_ShowString(0,12*11,12,RED,   "    Cleaing...     ");
						}
						{ // ���SWD�Ĵ���
							writeDP( DP_ABORT,0x1F);
							stm32f2Protection( DISABLE,DISABLE );
						}
						OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
						OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
						
						LCD_ShowString(0,12*10,12,RED, "  Protection Cleared ");
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*11,12,RED, "    ��ϵ���������   ");
						}
						else
						{
							LCD_ShowString(0,12*11,12,RED, "Download again without power");
						}
												
						return 0xFFBB;
					}
			}
	}
	
	{ // ���Flash
		if( userfile.ChipEraseWay == 00 )  // ȫ����
		{
			writeAP( AP_CSW,0x23000002);   // 32λд����
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0x45670123 );
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0xCDEF89AB );
			
			writeAP( AP_CSW,0x00000002);   // ��
			u32 readWord0 = 0;
			do
			{
				writeAP(AP_TAR, (u32)&F4_FLASH->SR);
				readAP(AP_DRW,&readWord0);
				readDP(DP_RDBUFF,&readWord0);
			}while((readWord0>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
			
//			if( F4FlashMemorySize > 0x400 )  // ȫ����������Ҫע��������С   ---  �����ǲ���д���ˣ�
//			{
//				writeAP( AP_CSW,0x23000002);   // д32λ
//				writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // ����������ͬʱ����PSIZEֵΪ2����ʾΪ32λ������С��
//				writeAP( AP_DRW, 0x0204 );
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->CR );
//				writeAP( AP_DRW, 0x0204 + ( 1<<16 )); // ��ʼ����
//				
//				
//				writeAP( AP_CSW,0x00000002);   // ��
//				u32 readWord1 = 0;
//				do
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//					readAP(AP_DRW,&readWord1);
//					readDP(DP_RDBUFF,&readWord1);
//				}while((readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
//			}
//			else
//			{	
				writeAP( AP_CSW,0x23000002);   // д32λ
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // ����������ͬʱ����PSIZEֵΪ2����ʾΪ32λ������С��
				writeAP( AP_DRW, 0x0204 );   // ����Ĳ�����һ����
				
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );
				writeAP( AP_DRW, 0x0204 + ( 1<<16 )); // ��ʼ����
				
				
				writeAP( AP_CSW,0x00000002);   // ��
				u32 readWord1 = 0;
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord1);
					readDP(DP_RDBUFF,&readWord1);
				}while((readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
//			}
		}
		else	// ���������ΪС�����������ʹ����������Ǵ�����Ҳ������1KB�����������������Լ򻯣������˷�ʱ�䣬��һ���汾����̫���⣡
					// ����֮�⣬���й������ڵĵ�ַ��
					// ע��F4�Ŀ����⡣��Ϊ������ȣ����Ի�Ƚϲ�һ�������Կ����ñ�ṹ��
		{
			u32 F4FlashChart[12][2] = {
																	{ 0x08000000,0x08003FFF },  // 0
																	{ 0x08004000,0x08007FFF },  // 1
																	{ 0x08008000,0x0800bFFF },
																	{ 0x0800c000,0x0800fFFF },
																	{ 0x08010000,0x0801fFFF },
																	{ 0x08020000,0x0803fFFF },
																	{ 0x08040000,0x0805fFFF },
																	{ 0x08060000,0x0807fFFF },
																	{ 0x08080000,0x0809fFFF },
																	{ 0x080A0000,0x080BfFFF },
																	{ 0x080C0000,0x080DfFFF },
																	{ 0x080E0000,0x080FfFFF },
																};
			
			u32 startAddressTmp0 = 0x08000000; // ��ʼҳ���ڵ�ַ
			u32 endAddressTmp1 = 0x08000000;   // ����ҳ���ڵ�ַ
			u32 startAddressPage = 0;
			u32 endAddressPage = 0;
																
			u32 startRollingAddressTmp3 = 0x08000000;
			u32 endRollingAddressTmp4 = 0x08000000;
			u32 startRollingAddressPage = 0;
			u32 endRollingdressPage = 0;
																
			for( u32 tmp5 = 0;tmp5<1024;tmp5++ ) // 2048�����Ϊ2M��С��
			{
				if( userfile.ProgramStartAddress < ( 0x08000000 + 0x400 * tmp5 ) )
				{
					startAddressTmp0 = 0x08000000 + 0x400 * (tmp5 - 1);
					break;
				}
			}
			for( u32 tmp6 = 0;tmp6<1024;tmp6++ ) // 
			{
				if( (userfile.ProgramStartAddress + userfile.ProgramSize) < ( 0x08000000 + 0x400 * tmp6 ) )
				{
					endAddressTmp1 = 0x08000000 + 0x400 * (tmp6 - 1);
					break;
				}
			}
			
			for( u32 tmp7 = 0;tmp7<1024;tmp7++ ) // 
			{
				if( userfile.RollingCodeStartAddress < ( 0x08000000 + 0x400 * tmp7 ) )
				{
					startRollingAddressTmp3 = 0x08000000 + 0x400 * (tmp7 - 1);
					break;
				}
			}
			for( u32 tmp8 = 0;tmp8<1024;tmp8++ ) // 
			{
				if( (userfile.RollingCodeStartAddress + 32 ) < ( 0x08000000 + 0x400 * tmp8 ) )  // �����SIZEֻ��32
				{
					endRollingAddressTmp4 = 0x08000000 + 0x400 * (tmp8 - 1);
					break;
				}
			}
			{ // ���������ڵ�ҳ��
				for( u8 Temp0 = 0;Temp0<4;Temp0++ ) // �ܹ�����4��
				{
					for( u8 Temp1 = 0;Temp1<12;Temp1++ )
					{
						if( Temp0 == 0 )
						{
							if( ( startAddressTmp0 >= F4FlashChart[Temp1][0] ) && ( startAddressTmp0 <= F4FlashChart[Temp1][1] ))
							{
								startAddressPage = Temp1;
								break;
							}
						}
						else if( Temp0 == 1)
						{
							if( ( endAddressTmp1 >= F4FlashChart[Temp1][0] ) && ( endAddressTmp1 <= F4FlashChart[Temp1][1] ))
							{
								endAddressPage = Temp1;
								break;
							}
						}
						else if( Temp0 == 2)
						{
							if( ( startRollingAddressTmp3 >= F4FlashChart[Temp1][0] ) && ( startRollingAddressTmp3 <= F4FlashChart[Temp1][1] ))
							{
								startRollingAddressPage = Temp1;
								break;
							}
						}
						else if( Temp0 == 3)
						{
							if( ( endRollingAddressTmp4 >= F4FlashChart[Temp1][0] ) && ( endRollingAddressTmp4 <= F4FlashChart[Temp1][1] ))
							{
								endRollingdressPage = Temp1;
								break;
							}
						}
					}
				}
			}
			{ // ��SWD���Flash����
				writeAP( AP_CSW,0x23000002);   // 32λд����
			
				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
				writeAP( AP_DRW, 0x45670123 );
				
				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
				writeAP( AP_DRW, 0xCDEF89AB );
				
				writeAP( AP_CSW,0x00000002);   // ��
				u32 readWord0 = 0;
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord0);
					readDP(DP_RDBUFF,&readWord0);
				}while((readWord0>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
				
				
				
				for( u8 i = startAddressPage; i<= endAddressPage; i++ )
				{
					writeAP( AP_CSW, 0x23000002 );   // д32λ
					writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // ����������ͬʱ����PSIZEֵΪ2����ʾΪ32λ������С��
//					if( i<12 )
//					{
						writeAP( AP_DRW, 0x0202 + (i<<3) );  // ��������������ҳ��
						writeAP( AP_TAR, (u32)&F4_FLASH->CR );
						writeAP( AP_DRW, 0x0202 + (i<<3) + (1<<16) ); // ��ʼ����
//					}
//					else
//					{
//						writeAP( AP_DRW, 0x0202 + ((i+4)<<3) );  // ��������������ҳ��
//						writeAP( AP_TAR, (u32)&F4_FLASH->CR );
//						writeAP( AP_DRW, 0x0202 + ((i+4)<<3) + (1<<16) ); // ��ʼ����
//					}
					
					
					writeAP( AP_CSW,0x00000002);   // ��
					u32 readWord1 = 0;
					do
					{
						writeAP(AP_TAR, (u32)&F4_FLASH->SR);
						readAP(AP_DRW,&readWord1);
						readDP(DP_RDBUFF,&readWord1);
					}while((readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
				}
				
				if( userfile.RollingCodeFunction == 0x55 ) // ʹ�ܹ��������ֻ�е��������ʹ�ܵ�ʱ���ٲ�������ֹ�����˴���ĵ�ַ��
				{
					for( u8 i = startRollingAddressPage; i<= endRollingdressPage; i++ )
					{
						writeAP( AP_CSW,0x23000002 );   // д32λ
						writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // ����������ͬʱ����PSIZEֵΪ2����ʾΪ32λ������С��
//						if( i<12 )
//						{
							writeAP( AP_DRW, 0x0202 + (i<<3) );  // ��������������ҳ��
							writeAP( AP_TAR, (u32)&F4_FLASH->CR );
							writeAP( AP_DRW, 0x0202 + (i<<3) + (1<<16) ); // ��ʼ����
//						}
//						else
//						{
//							writeAP( AP_DRW, 0x0202 + ((i+4)<<3) );  // ��������������ҳ��
//							writeAP( AP_TAR, (u32)&F4_FLASH->CR );
//							writeAP( AP_DRW, 0x0202 + ((i+4)<<3) + (1<<16) ); // ��ʼ����
//						}
						
						writeAP( AP_CSW,0x00000002);   // ��
						u32 readWord1 = 0;
						do
						{
							writeAP(AP_TAR, (u32)&F4_FLASH->SR);
							readAP(AP_DRW,&readWord1);
							readDP(DP_RDBUFF,&readWord1);
						}while((readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
					}
				}
				
				{ // ����Flash
					writeAP( AP_CSW,0x23000002 );   // д32λ
					writeAP( AP_TAR, (u32)&F4_FLASH->CR );
					writeAP( AP_DRW, 0x80000000 );
				}
			}
		}
	}
	{ // дFlash��ͬʱҪд�����������
		
		{  // ����Flash����
			writeAP( AP_CSW,0x23000002);   // 32λд����
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0x45670123 );
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0xCDEF89AB );
			
			writeAP( AP_CSW,0x00000002);   // ��
			u32 readWord0 = 0;
			do
			{
				writeAP(AP_TAR, (u32)&F4_FLASH->SR);
				readAP(AP_DRW,&readWord0);
				readDP(DP_RDBUFF,&readWord0);
			}while((readWord0>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
		}
		{ // ʹ�ܱ༭λ
			writeAP( AP_TAR, (u32)&F4_FLASH->CR );
			writeAP(AP_DRW, 0x0201 );  // 32��ȣ�PGʹ��
		}
		
		{ // д������
			u32 HowMany4K = 0;
			if( userfile.ProgramSize % 0x1000 != 0 )
				HowMany4K = userfile.ProgramSize/0x1000 + 1;
			else
				HowMany4K = userfile.ProgramSize/0x1000;
			
			for( u32 tmp11 = 0;tmp11<HowMany4K; tmp11++ )  // дFlash�Ĳ�����ÿ�β���4K�ֽ�
			{
				u32 Address = userfile.ProgramStartAddress + tmp11*0x1000;  // ��ʼ������ַ
				W25QXX_Read( W25Q4KBuf.databuf4K , userfile.ProgramSaveAtW25QAddress + tmp11*0x1000, 0x1000); // 4k��С��16�����£�����0x1000
				
				writeAP( AP_CSW,0x23000002 | 0x10 ); // ����Ϊ32bit,д���ҵ�ַ�Զ����ӣ�
				writeAP( AP_TAR, Address );  // ��ַ��
				for( u32 tmp12 = 0;tmp12 <0x1000; tmp12 += 4 ) // д��4K
				{
					{ // ʵ�⣬���԰�����ĸ�ʡ�Ե����������������ٶ�һ�������ǣ���56S - 36S����Ϊд�������ܸ�������Ҫ���������������ʱ�������պ��ֹ�ٶ�������Ӱ���ٶȣ�
//								writeAP( AP_CSW,0x00000002 );   // ��
//								do
//								{
//									writeAP(AP_TAR, (u32)&FLASH->SR.All);
//									readAP(AP_DRW,&readWord0);
//									readDP(DP_RDBUFF,&readWord0);
//								}while(readWord0&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
					}
					// ����ķ��������Ը�Ϊ4K�Զ����ӵģ�������Ҫ��ʱ�䣬������ʱ������������ٶȣ�Ҳ���Ǻ�����������ʱ�Ȳ����ǣ��պ�Ϊ�������ٶȣ��������������
					if( ((Address + tmp12) %0x1000) == 0 ) // ˵������4k�ٽ�㣬��Ҫ����д��ַ
					{
						writeAP( AP_TAR, Address + tmp12 );  // ��ַ��
					}
					
//					if( (Address+tmp12)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
//					{
						writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+3] << 24) + ( W25Q4KBuf.databuf4K[tmp12+2] << 16 ) + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
//					}
//					else
//					{
//						writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
//					}
				}
				{ // ��������������������4K��С�������ݶ���10%Ϊ���ȣ�
					// ���ʹ��emWinӦ����һ���򵥵����⣡
					float tmp13 = (float)((float)((float)tmp11+1.0)/(float)HowMany4K);
					for( float tmp14 = 0.0;tmp14 <= (float)1.0; tmp14 += 0.1 )
					{
						if( tmp13 <= tmp14 ) // �ҵ���һ�ο�ʼС�ڵ��ڵ�ֵ��
						{
							if( (tmp14 > 0.00) && ( tmp14 <= 0.1) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >             10%");
								break;
							}
							else if( (tmp14 > 0.1) && (tmp14 <=0.2 ) )
							{
								LCD_ShowString(0,12*10,12, BLACK,"   >>            20%");
								break;
							}
							else if( (tmp14 > 0.2) && (tmp14 <=0.3) )
							{
								LCD_ShowString(0,12*10,12, BLACK,"   >>>           30%");
								break;
							}
							else if( (tmp14 > 0.3) && (tmp14 <=0.4) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>          40%");
								break;
							}
							else if( (tmp14 > 0.4) && (tmp14 <=0.5) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>         50%");
								break;
							}
							else if( (tmp14 > 0.5) && (tmp14 <=0.6) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>        60%");
								break;
							}
							else if( (tmp14 > 0.6) && (tmp14 <=0.7) )
							{
								LCD_ShowString(0,12*10,12, BLACK,"   >>>>>>>       70%");
								break;
							}
							else if( (tmp14 > 0.7) && (tmp14 <=0.8) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>      80%");
								break;
							}
							else if( (tmp14 > 0.8) && (tmp14 <=0.9) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>>     90%");
								break;
							}
//									else if( (tmp14 > 0.9) && (tmp14 <=2.1) )
//									{
//										LCD_ShowString(0,12*10,12, "   >>>>>>>>>>    100%");
//										break;
//									}
						}
					}
					if( tmp11 + 1 >= HowMany4K )
					{
						LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>>>   100%");
					}
				}
			}
		}
		{ // д������
			u32 programTimesTmp15 = 0;  // ʵ���Ѿ���̴���
			u32 valueTmp16 = 0; 				// ʵ��д��Ĺ���ֵ
			if( userfile.RollingCodeFunction == 0x55 ) // ��ʾ��Ҫ����������
			{
				if( userfile.DownloaderNumbers[255] != 0xFF )
				{
					programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + 256;
				}
				else
				{
					for( u16 Tmp16 = 0;Tmp16<256;Tmp16++ )
					{
						if( userfile.DownloaderNumbers[Tmp16] == 0xFF )
						{
							programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + Tmp16;  // ʵ�ʱ�̴���
							break;
						}
					}
				}
				valueTmp16 = userfile.RollingCodeStartValue + ( userfile.RollingCodeStepValue* programTimesTmp15 );
				writeAP( AP_TAR, userfile.RollingCodeStartAddress );  // ��ַ��
//				{
//					if( (userfile.RollingCodeStartAddress)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
//					{
						writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//					}
//					else
//					{
//						writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//					}
//				}
//				{
//					writeAP( AP_TAR, userfile.RollingCodeStartAddress+2 );  // ��ַ��
//					{
//						if( (userfile.RollingCodeStartAddress+2)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
//						{
//							writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
//						}
//						else
//						{
//							writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//						}
//					}
//				}
			
				{ // ���������
					u32 apId = 0;
					writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
					writeAP( AP_CSW,0x2 | 0x10);   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
					writeAP( AP_TAR,userfile.RollingCodeStartAddress);   // дĿ���ַ
					readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
					readDP( DP_RDBUFF, &apId );
					if( apId != valueTmp16 )
					{
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*11,12,RED, "    ����У��ʧ��     ");
						}
						else
						{
							LCD_ShowString(0,12*11,12,RED, "Roll code check failed");
						}
						returnNumber = 0x00;  // ������
						return returnNumber;
					}
				}
			}
		}
		{ // У�����������ʱֻʹ�ú�У�飡
			switch( userfile.FlashCheckWay )
			{
				case 0x00: // ������
					if( L_ENGLISH_VER != 0x55AA )
					{
						LCD_ShowString(0,12*11,12,RED, "     ��У��FLASH     ");
					}
					else
					{
						LCD_ShowString(0,12*11,12,RED, "   not check FLASH   ");
					}
					returnNumber = 0x55AA;
					break;
				case 0x01: // �������
					break;
				case 0x02: // CRCУ�飨��У�飩 -- �պ�Ӧ��ֻ��������У�鷽ʽ����Ϊ������Ҳû�б�Ҫ���ˣ�
					{ // ������У�鷽ʽ��������Ҫ����ʼ��ַ4�ֽڶ��룬�����С4�ֽڱ���������պ����Ҫ��֤�������֤���ˣ����Դ�����Ͻ��������⣡
						u16 sumTmp15 = 0;  // ���ĺ�У�����
						writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
						writeAP( AP_CSW,0x2 | 0x10);   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
						u32 addressTmp16 = userfile.ProgramStartAddress;
						writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
						
						u32 apId = 0;
						readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
						
						for( ;addressTmp16 < ( userfile.ProgramStartAddress + userfile.ProgramSize );addressTmp16 += 4 )
						{
//									u32 apId = 0;
//									if( ( addressTmp16 %0x1000 ) == 0 )
//									{
//										writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
//									}
//									readAP( AP_DRW,&apId);  // ���ڶ����ݵĻ���ÿ��ֻ�ܶ���һ�εġ�������һ����ʱ��
//									readDP( DP_RDBUFF, &apId );
//									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
							if( ( addressTmp16 %0x1000 ) == 0 )
							{
								writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
								readAP( AP_DRW,&apId); 
							}
							readAP( AP_DRW,&apId);  // ����������ϴεġ�����Ҳ����Ҫ�ġ�
							sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
						}
						if( sumTmp15 == userfile.SumValueOfProgram )
						{
							if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12, BLACK,"   FLASH У��ɹ�   ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check success ");
									}
							returnNumber = 0x55AA;
						}
						else
						{
							if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12,BLACK, "   FLASH У��ʧ��    ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check failure ");
									}
							returnNumber = 0x1122;
						}
					}
					break;
				case 0x03: // MD5ֵ����
					break;
				default:
					break;
			}
		}
		{ // д���룬��У�����������
			// Ϊ��ʹ��������ȽϷ��㣺Ҫ��1��������4�ֽڶ����ַ�ϣ�����ʹ��С��ģʽ��
			{ // ��������ַ
				
			}
			{ // д����
				
			}
			{ // У����룬ʹ��ÿ���Աȣ�
				
			}
		}
		{ // ������������(��ʵ����Ӧ�ü�һ����������ʹ�ܵĲ�����)
			switch( userfile.OptionBytesFunction )
			{
				case 0x55: // ʹ��ѡ���ֽڣ��պ��ܿ��ţ�
					break;
				case 0xAA: // ʹ�ܶ�����
					stm32f2Protection( ENABLE,DISABLE );
					break;
				case 0x11: // ʹ��д����
					stm32f2Protection( DISABLE,ENABLE );
					break;
				case 0x1A: // ʹ�ܶ�д����
					stm32f2Protection( ENABLE,ENABLE );
					break;
			}
		}
		{
			writeAP( AP_CSW,0x23000002); // д
			writeAP(AP_TAR, (u32)&F4_FLASH->CR);
			writeAP(AP_DRW, 0x80000000);  // ��ֹ��̺�����
		}
		{
			resetAndHaltTarget( );
			runTarget( );
		}
	}
	return returnNumber;
}


u32 DownloadFlashOfSTM32F4xx( _FileInformation userfile )
{
	u32 returnNumber = 0;
	u32 F4FlashMemorySize = 0;  // �ڲ������������ͱ�������ʱ������Ҫ֪�����С����������û�ж����ֵ��
	{ // 
		{ // ���Flash�Ĵ�С��
			u32 apId = 0;
			writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
			writeAP( AP_CSW,0x2 );   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
			writeAP( AP_TAR,(u32)(0x1FFF7A22));   // дĿ���ַ
			readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
			readDP( DP_RDBUFF, &apId );
			F4FlashMemorySize = apId>>16;
			{
				if( userfile.ChipTypePrefix == 0x20)
				{
					F4FlashMemorySize = 0;
				}
			}
		}
		{ // ��鱣������
					OS_ERR err;
					u32 apId = 0;
					writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
					writeAP( AP_CSW,0x2 );   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
					writeAP( AP_TAR,(u32)&F4_FLASH->OPTCR);   // дĿ���ַ
					readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
					readDP( DP_RDBUFF, &apId );
			
			
					if( ((apId>>8)&(0xFF)) == 0xAA ) // û�м��������
					{
						{ // ���д����
							if( ((apId>>16)&(0x0FFF)) != 0x0FFF )  // ˵������д����
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*10,12,RED, "      ����д����      ");
								}
								else
								{
									LCD_ShowString(0,12*10,12,RED, "  have write protect  ");
								}
								goto dealTmp0;
							}
							
							if( ( F4FlashMemorySize > 0x400 ) && (F4FlashMemorySize != 0xFFFF)) // ����1M�Ĵ洢��ʵ���з����ˣ�0xFFFF������һ�����ܵ�STM32F1��оƬҲ������������⣡
							{
								writeAP( AP_TAR,(u32)&F4_FLASH->OPTCR1);
								readAP( AP_DRW,&apId);
								readDP( DP_RDBUFF, &apId );
								if( ((apId>>16)&(0x0FFF)) != 0x0FFF )  // ˵������д����
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*10,12,RED, "      ����д����      ");
									}
									else
									{
										LCD_ShowString(0,12*10,12,RED, "  have write protect  ");
									}
									goto dealTmp0;
								}
							}
						}
					}
					else if( ((apId>>8)&(0xFF)) == 0xCC )  // �����Իָ��Ķ�����
					{
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*10,12,RED, "         ����        ");
							LCD_ShowString(0,12*11,12,RED, "       ������-LV2    ");
						}
						else
						{
							LCD_ShowString(0,12*10,12,RED, "       warning     ");
							LCD_ShowString(0,12*11,12,RED, "    ReadProtect-LV2  ");
						}
						return 0x00;
					}
					else  // �����˿�������Ķ�����
					{ // �����˶�������Ҫ�����
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*10,12,RED, "      ���ֶ�����      ");
						}
						else
						{
							LCD_ShowString(0,12*10,12,RED, "      read protect    ");
						}
						dealTmp0:
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*11,12,RED, "        �����.      ");
						}
						else
						{
							LCD_ShowString(0,12*11,12,RED, "      Cleaning...    ");
						}
						{ // ���SWD�Ĵ���
							writeDP( DP_ABORT,0x1F);
							if( userfile.ChipTypePrefix == 0x20 )
							{
								stm32f2Protection( DISABLE,DISABLE );
							}
							else
							{
								stm32f4Protection( DISABLE,DISABLE );
							}
						}
						OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
						
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*10,12,RED, "      ���������     ");
							LCD_ShowString(0,12*11,12,RED, "    ��ϵ���������   ");
						}
						else
						{
							LCD_ShowString(0,12*10,12,RED, " Protect cleared,Download again without power!");
						}
						return 0xFFBB;
					}
			}
	}
	
	{ // ���Flash
		if( userfile.ChipEraseWay == 00 )  // ȫ����
		{
			writeAP( AP_CSW,0x23000002);   // 32λд����
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0x45670123 );
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0xCDEF89AB );
			
			writeAP( AP_CSW,0x00000002);   // ��
			u32 readWord0 = 0;
			do
			{
				writeAP(AP_TAR, (u32)&F4_FLASH->SR);
				readAP(AP_DRW,&readWord0);
				readDP(DP_RDBUFF,&readWord0);
			}while((readWord0>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
			
// ע�⣺֮ǰ�ǡ��� F4FlashMemorySize > 0x400 û�з��ִ���
			if( F4FlashMemorySize <= 0x400 )  // ȫ����������Ҫע��������С
			{
				writeAP( AP_CSW,0x23000002);   // д32λ
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // ����������ͬʱ����PSIZEֵΪ2����ʾΪ32λ������С��
				writeAP( AP_DRW, 0x0204 );
				
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );
				writeAP( AP_DRW, 0x0204 + ( 1<<16 )); // ��ʼ����
				
				
				writeAP( AP_CSW,0x00000002);   // ��
				u32 readWord1 = 0;
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord1);
					readDP(DP_RDBUFF,&readWord1);
				}while((readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
			}
			else
			{	
				writeAP( AP_CSW,0x23000002);   // д32λ
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // ����������ͬʱ����PSIZEֵΪ2����ʾΪ32λ������С��
				writeAP( AP_DRW, 0x8204 );
				
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );
				writeAP( AP_DRW, 0x8204 + ( 1<<16 )); // ��ʼ����
				
				
				writeAP( AP_CSW,0x00000002);   // ��
				u32 readWord1 = 0;
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord1);
					readDP(DP_RDBUFF,&readWord1);
				}while((readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
			}
			
			{ // ����Flash,����û�м����������³��������⣡
				writeAP( AP_CSW,0x23000002 );   // д32λ
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );
				writeAP( AP_DRW, 0x80000000 );
			}
		}
		else	// ���������ΪС�����������ʹ����������Ǵ�����Ҳ������1KB�����������������Լ򻯣������˷�ʱ�䣬��һ���汾����̫���⣡
					// ����֮�⣬���й������ڵĵ�ַ��
					// ע��F4�Ŀ����⡣��Ϊ������ȣ����Ի�Ƚϲ�һ�������Կ����ñ�ṹ��
		{
			u32 F4FlashChart[24][2] = {
																	{ 0x08000000,0x08003FFF },  // 0
																	{ 0x08004000,0x08007FFF },  // 1
																	{ 0x08008000,0x0800bFFF },
																	{ 0x0800c000,0x0800fFFF },
																	{ 0x08010000,0x0801fFFF },
																	{ 0x08020000,0x0803fFFF },
																	{ 0x08040000,0x0805fFFF },
																	{ 0x08060000,0x0807fFFF },
																	{ 0x08080000,0x0809fFFF },
																	{ 0x080A0000,0x080BfFFF },
																	{ 0x080C0000,0x080DfFFF },
																	{ 0x080E0000,0x080FfFFF },
																	
																	{ 0x08100000,0x08103FFF },
																	{ 0x08104000,0x08107FFF },
																	{ 0x08108000,0x0810bFFF },
																	{ 0x0810c000,0x0810fFFF },
																	{ 0x08110000,0x0811fFFF },
																	{ 0x08120000,0x0813fFFF },
																	{ 0x08140000,0x0815fFFF },
																	{ 0x08160000,0x0817fFFF },
																	{ 0x08180000,0x0819fFFF },
																	{ 0x081A0000,0x081BfFFF },
																	{ 0x081C0000,0x081DfFFF },
																	{ 0x081E0000,0x081FfFFF },  // 23
																};
			
			u32 startAddressTmp0 = 0x08000000; // ��ʼҳ���ڵ�ַ
			u32 endAddressTmp1 = 0x08000000;   // ����ҳ���ڵ�ַ
			u32 startAddressPage = 0;
			u32 endAddressPage = 0;
																
			u32 startRollingAddressTmp3 = 0x08000000;
			u32 endRollingAddressTmp4 = 0x08000000;
			u32 startRollingAddressPage = 0;
			u32 endRollingdressPage = 0;
																
			for( u32 tmp5 = 0;tmp5<1024;tmp5++ ) // 1024�����Ϊ1M��С��
			{
				if( userfile.ProgramStartAddress < ( 0x08000000 + 0x400 * tmp5 ) )
				{
					startAddressTmp0 = 0x08000000 + 0x400 * (tmp5 - 1);
					break;
				}
			}
			for( u32 tmp6 = 0;tmp6<1024;tmp6++ ) // 
			{
				if( (userfile.ProgramStartAddress + userfile.ProgramSize) < ( 0x08000000 + 0x400 * tmp6 ) )
				{
					endAddressTmp1 = 0x08000000 + 0x400 * (tmp6 - 1);
					break;
				}
			}
			
			for( u32 tmp7 = 0;tmp7<1024;tmp7++ ) // 
			{
				if( userfile.RollingCodeStartAddress < ( 0x08000000 + 0x400 * tmp7 ) )
				{
					startRollingAddressTmp3 = 0x08000000 + 0x400 * (tmp7 - 1);
					break;
				}
			}
			for( u32 tmp8 = 0;tmp8<1024;tmp8++ ) // 
			{
				if( (userfile.RollingCodeStartAddress + 32 ) < ( 0x08000000 + 0x400 * tmp8 ) )  // �����SIZEֻ��32
				{
					endRollingAddressTmp4 = 0x08000000 + 0x400 * (tmp8 - 1);
					break;
				}
			}
			{ // ���������ڵ�ҳ��
				for( u8 Temp0 = 0;Temp0<4;Temp0++ ) // �ܹ�����4��
				{
					for( u8 Temp1 = 0;Temp1<24;Temp1++ )
					{
						if( Temp0 == 0 )
						{
							if( ( startAddressTmp0 >= F4FlashChart[Temp1][0] ) && ( startAddressTmp0 <= F4FlashChart[Temp1][1] ))
							{
								startAddressPage = Temp1;
								break;
							}
						}
						else if( Temp0 == 1)
						{
							if( ( endAddressTmp1 >= F4FlashChart[Temp1][0] ) && ( endAddressTmp1 <= F4FlashChart[Temp1][1] ))
							{
								endAddressPage = Temp1;
								break;
							}
						}
						else if( Temp0 == 2)
						{
							if( ( startRollingAddressTmp3 >= F4FlashChart[Temp1][0] ) && ( startRollingAddressTmp3 <= F4FlashChart[Temp1][1] ))
							{
								startRollingAddressPage = Temp1;
								break;
							}
						}
						else if( Temp0 == 3)
						{
							if( ( endRollingAddressTmp4 >= F4FlashChart[Temp1][0] ) && ( endRollingAddressTmp4 <= F4FlashChart[Temp1][1] ))
							{
								endRollingdressPage = Temp1;
								break;
							}
						}
					}
				}
			}
			{ // ��SWD���Flash����
				writeAP( AP_CSW,0x23000002);   // 32λд����
			
				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
				writeAP( AP_DRW, 0x45670123 );
				
				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
				writeAP( AP_DRW, 0xCDEF89AB );
				
				writeAP( AP_CSW,0x00000002);   // ��
				u32 readWord0 = 0;
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord0);
					readDP(DP_RDBUFF,&readWord0);
				}while((readWord0>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
				
				
				
				for( u8 i = startAddressPage; i<= endAddressPage; i++ )
				{
					writeAP( AP_CSW,0x23000002 );   // д32λ
					writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // ����������ͬʱ����PSIZEֵΪ2����ʾΪ32λ������С��
					if( i<12 )
					{
						writeAP( AP_DRW, 0x0202 + (i<<3) );  // ��������������ҳ��
						writeAP( AP_TAR, (u32)&F4_FLASH->CR );
						writeAP( AP_DRW, 0x0202 + (i<<3) + (1<<16) ); // ��ʼ����
					}
					else
					{
						writeAP( AP_DRW, 0x0202 + ((i+4)<<3) );  // ��������������ҳ��
						writeAP( AP_TAR, (u32)&F4_FLASH->CR );
						writeAP( AP_DRW, 0x0202 + ((i+4)<<3) + (1<<16) ); // ��ʼ����
					}
					
					
					writeAP( AP_CSW,0x00000002);   // ��
					u32 readWord1 = 0;
					do
					{
						writeAP(AP_TAR, (u32)&F4_FLASH->SR);
						readAP(AP_DRW,&readWord1);
						readDP(DP_RDBUFF,&readWord1);
					}while((readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
				}
				
				if( userfile.RollingCodeFunction == 0x55 ) // ʹ�ܹ��������ֻ�е��������ʹ�ܵ�ʱ���ٲ�������ֹ�����˴���ĵ�ַ��
				{
					for( u8 i = startRollingAddressPage; i<= endRollingdressPage; i++ )
					{
						writeAP( AP_CSW,0x23000002 );   // д32λ
						writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // ����������ͬʱ����PSIZEֵΪ2����ʾΪ32λ������С��
						if( i<12 )
						{
							writeAP( AP_DRW, 0x0202 + (i<<3) );  // ��������������ҳ��
							writeAP( AP_TAR, (u32)&F4_FLASH->CR );
							writeAP( AP_DRW, 0x0202 + (i<<3) + (1<<16) ); // ��ʼ����
						}
						else
						{
							writeAP( AP_DRW, 0x0202 + ((i+4)<<3) );  // ��������������ҳ��
							writeAP( AP_TAR, (u32)&F4_FLASH->CR );
							writeAP( AP_DRW, 0x0202 + ((i+4)<<3) + (1<<16) ); // ��ʼ����
						}
						
						writeAP( AP_CSW,0x00000002);   // ��
						u32 readWord1 = 0;
						do
						{
							writeAP(AP_TAR, (u32)&F4_FLASH->SR);
							readAP(AP_DRW,&readWord1);
							readDP(DP_RDBUFF,&readWord1);
						}while((readWord1>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
					}
				}
				
				{ // ����Flash
					writeAP( AP_CSW,0x23000002 );   // д32λ
					writeAP( AP_TAR, (u32)&F4_FLASH->CR );
					writeAP( AP_DRW, 0x80000000 );
				}
			}
		}
	}
	{ // дFlash��ͬʱҪд�����������
		
		{  // ����Flash����
			writeAP( AP_CSW,0x23000002);   // 32λд����
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0x45670123 );
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0xCDEF89AB );
			
			writeAP( AP_CSW,0x00000002);   // ��
			u32 readWord0 = 0;
			do
			{
				writeAP(AP_TAR, (u32)&F4_FLASH->SR);
				readAP(AP_DRW,&readWord0);
				readDP(DP_RDBUFF,&readWord0);
			}while((readWord0>>16)&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
		}
		{ // ʹ�ܱ༭λ
			writeAP( AP_TAR, (u32)&F4_FLASH->CR );
			writeAP(AP_DRW, 0x0201 );  // 32��ȣ�PGʹ��
		}
		
		{ // д������
			u32 HowMany4K = 0;
			if( userfile.ProgramSize % 0x1000 != 0 )
				HowMany4K = userfile.ProgramSize/0x1000 + 1;
			else
				HowMany4K = userfile.ProgramSize/0x1000;
			
			for( u32 tmp11 = 0;tmp11<HowMany4K; tmp11++ )  // дFlash�Ĳ�����ÿ�β���4K�ֽ�
			{
				u32 Address = userfile.ProgramStartAddress + tmp11*0x1000;  // ��ʼ������ַ
				W25QXX_Read( W25Q4KBuf.databuf4K , userfile.ProgramSaveAtW25QAddress + tmp11*0x1000, 0x1000); // 4k��С��16�����£�����0x1000
				
				writeAP( AP_CSW,0x23000002 | 0x10 ); // ����Ϊ32bit,д���ҵ�ַ�Զ����ӣ�
				writeAP( AP_TAR, Address );  // ��ַ��
				for( u32 tmp12 = 0;tmp12 <0x1000; tmp12 += 4 ) // д��4K
				{
					{ // ʵ�⣬���԰�����ĸ�ʡ�Ե����������������ٶ�һ�������ǣ���56S - 36S����Ϊд�������ܸ�������Ҫ���������������ʱ�������պ��ֹ�ٶ�������Ӱ���ٶȣ�
//								writeAP( AP_CSW,0x00000002 );   // ��
//								do
//								{
//									writeAP(AP_TAR, (u32)&FLASH->SR.All);
//									readAP(AP_DRW,&readWord0);
//									readDP(DP_RDBUFF,&readWord0);
//								}while(readWord0&0x1); // ��Ϊ1��ʱ��˵��û�в�����ɣ�
					}
					// ����ķ��������Ը�Ϊ4K�Զ����ӵģ�������Ҫ��ʱ�䣬������ʱ������������ٶȣ�Ҳ���Ǻ�����������ʱ�Ȳ����ǣ��պ�Ϊ�������ٶȣ��������������
					if( ((Address + tmp12) %0x1000) == 0 ) // ˵������4k�ٽ�㣬��Ҫ����д��ַ
					{
						writeAP( AP_TAR, Address + tmp12 );  // ��ַ��
					}
					
//					if( (Address+tmp12)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
//					{
						writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+3] << 24) + ( W25Q4KBuf.databuf4K[tmp12+2] << 16 ) + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
//					}
//					else
//					{
//						writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
//					}
				}
				{ // ��������������������4K��С�������ݶ���10%Ϊ���ȣ�
					// ���ʹ��emWinӦ����һ���򵥵����⣡
					float tmp13 = (float)((float)((float)tmp11+1.0)/(float)HowMany4K);
					for( float tmp14 = 0.0;tmp14 <= (float)1.0; tmp14 += 0.1 )
					{
						if( tmp13 <= tmp14 ) // �ҵ���һ�ο�ʼС�ڵ��ڵ�ֵ��
						{
							if( (tmp14 > 0.00) && ( tmp14 <= 0.1) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >             10%");
								break;
							}
							else if( (tmp14 > 0.1) && (tmp14 <=0.2 ) )
							{
								LCD_ShowString(0,12*10,12, BLACK,"   >>            20%");
								break;
							}
							else if( (tmp14 > 0.2) && (tmp14 <=0.3) )
							{
								LCD_ShowString(0,12*10,12, BLACK,"   >>>           30%");
								break;
							}
							else if( (tmp14 > 0.3) && (tmp14 <=0.4) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>          40%");
								break;
							}
							else if( (tmp14 > 0.4) && (tmp14 <=0.5) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>         50%");
								break;
							}
							else if( (tmp14 > 0.5) && (tmp14 <=0.6) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>        60%");
								break;
							}
							else if( (tmp14 > 0.6) && (tmp14 <=0.7) )
							{
								LCD_ShowString(0,12*10,12, BLACK,"   >>>>>>>       70%");
								break;
							}
							else if( (tmp14 > 0.7) && (tmp14 <=0.8) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>      80%");
								break;
							}
							else if( (tmp14 > 0.8) && (tmp14 <=0.9) )
							{
								LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>>     90%");
								break;
							}
//									else if( (tmp14 > 0.9) && (tmp14 <=2.1) )
//									{
//										LCD_ShowString(0,12*10,12, "   >>>>>>>>>>    100%");
//										break;
//									}
						}
					}
					if( tmp11 + 1 >= HowMany4K )
					{
						LCD_ShowString(0,12*10,12,BLACK, "   >>>>>>>>>>   100%");
					}
				}
			}
		}
		{ // д������
			u32 programTimesTmp15 = 0;  // ʵ���Ѿ���̴���
			u32 valueTmp16 = 0; 				// ʵ��д��Ĺ���ֵ
			if( userfile.RollingCodeFunction == 0x55 ) // ��ʾ��Ҫ����������
			{
				if( userfile.DownloaderNumbers[255] != 0xFF )
				{
					programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + 256;
				}
				else
				{
					for( u16 Tmp16 = 0;Tmp16<256;Tmp16++ )
					{
						if( userfile.DownloaderNumbers[Tmp16] == 0xFF )
						{
							programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + Tmp16;  // ʵ�ʱ�̴���
							break;
						}
					}
				}
				valueTmp16 = userfile.RollingCodeStartValue + ( userfile.RollingCodeStepValue* programTimesTmp15 );
				writeAP( AP_TAR, userfile.RollingCodeStartAddress );  // ��ַ��
//				{
//					if( (userfile.RollingCodeStartAddress)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
//					{
						writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//					}
//					else
//					{
//						writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//					}
//				}
//				{
//					writeAP( AP_TAR, userfile.RollingCodeStartAddress+2 );  // ��ַ��
//					{
//						if( (userfile.RollingCodeStartAddress+2)%4 != 0 ) // ��ʾ����4�ı�������ȡ��16λ�ڸ�λ
//						{
//							writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
//						}
//						else
//						{
//							writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//						}
//					}
//				}
			
				{ // ���������
					u32 apId = 0;
					writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
					writeAP( AP_CSW,0x2 | 0x10);   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
					writeAP( AP_TAR,userfile.RollingCodeStartAddress);   // дĿ���ַ
					readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
					readDP( DP_RDBUFF, &apId );
					if( apId != valueTmp16 )
					{
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*11,12,RED, "    ����У��ʧ��     ");
						}
						else
						{
							LCD_ShowString(0,12*11,12,RED, "Roll code check failed");
						}
						returnNumber = 0x00;  // ������
						return returnNumber;
					}
				}
			}
		}
		{ // У�����������ʱֻʹ�ú�У�飡
			switch( userfile.FlashCheckWay )
			{
				case 0x00: // ������
					if( L_ENGLISH_VER != 0x55AA )
					{
						LCD_ShowString(0,12*11,12,RED, "     ��У��FLASH     ");
					}
					else
					{
						LCD_ShowString(0,12*11,12,RED, "   not check FLASH   ");
					}
					returnNumber = 0x55AA;
					break;
				case 0x01: // �������
					break;
				case 0x02: // CRCУ�飨��У�飩 -- �պ�Ӧ��ֻ��������У�鷽ʽ����Ϊ������Ҳû�б�Ҫ���ˣ�
					{ // ������У�鷽ʽ��������Ҫ����ʼ��ַ4�ֽڶ��룬�����С4�ֽڱ���������պ����Ҫ��֤�������֤���ˣ����Դ�����Ͻ��������⣡
						u16 sumTmp15 = 0;  // ���ĺ�У�����
						writeDP( DP_SELECT, 0x00 );  // ���ʼĴ���
						writeAP( AP_CSW,0x2 | 0x10);   // ����Ϊ32bit��ͬʱ��ַ�Զ�����
						u32 addressTmp16 = userfile.ProgramStartAddress;
						writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
						
						u32 apId = 0;
						readAP( AP_DRW,&apId);  // ��һ�����ݣ���Ҫ��
						
						for( ;addressTmp16 < ( userfile.ProgramStartAddress + userfile.ProgramSize );addressTmp16 += 4 )
						{
//									u32 apId = 0;
//									if( ( addressTmp16 %0x1000 ) == 0 )
//									{
//										writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
//									}
//									readAP( AP_DRW,&apId);  // ���ڶ����ݵĻ���ÿ��ֻ�ܶ���һ�εġ�������һ����ʱ��
//									readDP( DP_RDBUFF, &apId );
//									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
							if( ( addressTmp16 %0x1000 ) == 0 )
							{
								writeAP( AP_TAR,addressTmp16);   // дĿ���ַ
								readAP( AP_DRW,&apId); 
							}
							readAP( AP_DRW,&apId);  // ����������ϴεġ�����Ҳ����Ҫ�ġ�
							sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
						}
						if( sumTmp15 == userfile.SumValueOfProgram )
						{
							if( L_ENGLISH_VER != 0x55AA )
							{
								LCD_ShowString(0,12*11,12, BLACK,"   FLASH У��ɹ�   ");
							}
								else
							{
								LCD_ShowString(0,12*11,12, BLACK," FLASH Check success ");
							}
							returnNumber = 0x55AA;
						}
						else
						{
							if( L_ENGLISH_VER != 0x55AA )
							{
								LCD_ShowString(0,12*11,12,BLACK, "   FLASH У��ʧ��    ");
							}
								else
							{
								LCD_ShowString(0,12*11,12, BLACK," FLASH Check failure ");
							}
							returnNumber = 0x1122;
							return returnNumber;  // ��ʱ��Ϊ��ʧ�ܣ����ء�
						}
					}
					break;
				case 0x03: // MD5ֵ����
					break;
				default:
					break;
			}
		}
		{ // д���룬��У�����������
			// Ϊ��ʹ��������ȽϷ��㣺Ҫ��1��������4�ֽڶ����ַ�ϣ�����ʹ��С��ģʽ��
			{ // ��������ַ
				
			}
			{ // д����
				
			}
			{ // У����룬ʹ��ÿ���Աȣ�
				
			}
		}
		{ // ������������(��ʵ����Ӧ�ü�һ����������ʹ�ܵĲ�����)
			switch( userfile.OptionBytesFunction )
			{
				case 0x55: // ʹ��ѡ���ֽڣ��պ��ܿ��ţ�
					break;
				case 0xAA: // ʹ�ܶ�����
					if( userfile.ChipTypePrefix == 0x20 )
							{
								stm32f2Protection( ENABLE,DISABLE );
							}
							else
							{
								stm32f4Protection( ENABLE,DISABLE );
							}
					break;
				case 0x11: // ʹ��д����
					if( userfile.ChipTypePrefix == 0x20 )
							{
								stm32f2Protection( DISABLE,ENABLE );
							}
							else
							{
								stm32f4Protection( DISABLE,ENABLE );
							}
					break;
				case 0x1A: // ʹ�ܶ�д����
					if( userfile.ChipTypePrefix == 0x20 )
							{
								stm32f2Protection( ENABLE,ENABLE );
							}
							else
							{
								stm32f4Protection( ENABLE,ENABLE );
							}
					break;
			}
		}
		{
			writeAP( AP_CSW,0x23000002); // д
			writeAP(AP_TAR, (u32)&F4_FLASH->CR);
			writeAP(AP_DRW, 0x80000000);  // ��ֹ��̺�����
		}
		{
			resetAndHaltTarget( );
			runTarget( );
		}
	}
	return returnNumber;
}

/**
 *  @B ����Flash���ռ�����
 *  @Attention ������ֵΪ0x55AA��ʱ����Ϊ�����Ѿ��ɹ�����ʱ��Ҫ���³�������ش�����(�����Ǹ��������ش������ġ�)
                         0x1122��ʱ�򣬱�ʾ����ʧ�ܣ������պ���Ҫ����Ҫ��������룬���պ�V2.0�汾��������
 */

u32 DownloadFlash( _FileInformation userfile )
{
	u8 forDownloaderAgain = 1;
	OS_ERR err;
	u32 dpID = 0;
	u32 apID = 0;
	reDownloader:
	
	if( filex.DownloaderNumbers[4] != 0x68 )
	{
		return 0;  // û�����ش����ˡ�
	}
	if( userfile.ChipTypePrefix != 0x60 )  // ��ʾ����STM8������
	{
		SWDInit();  // ��������г�ʼ��������
		
		SWD_RESET_PIN = 0;  // ʵ���е�SWD ResetPinӦ����ʲôλ�ñȽ���Ҫ����Ҫʵ�ʽ��в��Բſ�����ɡ�
		if( HARDWARE_VERSION == MINI_VERSION )
		{
			GPIOB->ODR.OutputData6_RW = 0;  // PB6
			GPIOB->ODR.OutputData1_RW = 0;  // PB1
		}
		
		
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_NON_STRICT,&err);  // ��ʱ20msʹ��λ�����ȶ���
		dpID = initDp( );
		if( dpID == 0x00 || dpID == 0xFFFFFFFF )
		{
			if( L_ENGLISH_VER != 0x55AA )
			{
				LCD_ShowString(0,12+12*9,12,RED, " SWD Ŀ�� dpID ����  "  );
				LCD_ShowString(0,12+12*10,12,RED,"   ����Ӳ������    " );
			}
			else
			{
				LCD_ShowString(0,12+12*9,12,RED, "    SWD dpID error   "  );
				LCD_ShowString(0,12+12*10,12,RED,"    check hardware   " );
			}
			return 0x00;  // ��ȷ��оƬ��Ӧ����ID�ģ�������ΪоƬ�ڿ�ʼʱ������Ӵ���
		}
		
		apID = readApID( );
		if( apID == 0x00 || apID == 0xFFFFFFFF )
		{
			if( L_ENGLISH_VER != 0x55AA )
			{
				LCD_ShowString(0,12+12*9,12,RED, " SWD Ŀ�� apID ����  " );
				LCD_ShowString(0,12+12*10,12,RED,"   ����Ӳ������    " );
			}
			else
			{
				LCD_ShowString(0,12+12*9,12,RED, "    SWD apID error   "  );
				LCD_ShowString(0,12+12*10,12,RED,"    check hardware   " );
			}
			return 0x00;
		}
		
		haltTarget();   // ʵ�ⷢ�֣�ʹ����һ����ߣ�resetAndHaltTarget�����յĽ��û�й�ϵ�����ݲ��Է��֣��ڸ�λ֮�У�ʹinitDp��readApIDҲ���ԴﵽֹͣMCU�Ĺ��ܡ�����ȽϾ��ᡣ��Ҫʵ�⣡
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_NON_STRICT,&err);  // ��ʱ20msʹ��λ�����ȶ���
		
		if( HARDWARE_VERSION == MINI_VERSION )
		{
			GPIOB->ODR.OutputData6_RW = 1;  // PB6
			GPIOB->ODR.OutputData1_RW = 1;  // PB1
		}
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_NON_STRICT,&err);  // ��ʱ20msʹ��λ�����ȶ���
		
		
		SWD_RESET_PIN = 1;  // ���ʱ����Իָ��ˡ�
		
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_NON_STRICT,&err);  // ��ʱ20msʹ��λ�����ȶ���
		//	resetAndHaltTarget();  // 
	}
	
	u32 returnNumber = 0;
	{
		LCD_ShowString(0,12*10,12,BLACK, "                     ");
		LCD_ShowString(0,12*11,12,BLACK, "                     ");
	}
	{
		if( userfile.SWDLowSpeedEnable == 0x55 )
			slowModeEnable = ENABLE;
		else
			slowModeEnable = DISABLE;
	}
	switch( userfile.ChipTypePrefix )
	{
		case 0x00: // F0ϵ��
			{ // ����Ƿ����SWD����
				returnNumber = DownloadFlashOfSTM32F0xx( userfile );
				if( returnNumber == 0xFFBB && forDownloaderAgain == 1 )
				{
					forDownloaderAgain = 0;
					goto reDownloader;
				}
			}
			break;
		case 0x10:  // ����������ʱ����ΪSTM32F1ϵ�к�����
//		{
//			returnNumber = DownloadFlashOfSTM32F1xx( userfile );
//		}
		
		{ // ����Ƿ����SWD����
				returnNumber = DownloadFlashOfSTM32F1xx( userfile );
				if( returnNumber == 0xFFBB && forDownloaderAgain == 1 )
				{
					forDownloaderAgain = 0;
					goto reDownloader;
				}
			}
			break;
//		case 0x20:  // F2ϵ��
//			{ // ����Ƿ����SWD����
//				returnNumber = DownloadFlashOfSTM32F2xx( userfile );  // ���ݷ��֣�Ӧ���ǲ���������Ҫ������֤����
//			}
//			if( returnNumber == 0xFFBB && forDownloaderAgain == 1 )
//			{
//				forDownloaderAgain = 0;
//				goto reDownloader;
//			}
//			break;
		case 0x30:  // F3ϵ��
		returnNumber = DownloadFlashOfSTM32F3xx( userfile );
			break;
		case 0x20:  // F2ϵ��
		case 0x40:  // F4ϵ��
			returnNumber = DownloadFlashOfSTM32F4xx( userfile );
			if( returnNumber == 0xFFBB && forDownloaderAgain == 1 )
			{
				forDownloaderAgain = 0;
				goto reDownloader;
			}
			break;
		case 0x41:
			returnNumber = DownloadFlashOf_stm32f7xx( userfile );
			break;
		case 0x50: 
			{
				returnNumber = DownloadFlashOfSTM32L0xx( userfile );
				if( returnNumber == 0xFFBB && forDownloaderAgain == 1 )
				{
					forDownloaderAgain = 0;
					goto reDownloader;
				}
			}
			break;
		case 0x51:
			returnNumber = DownloadFlashOfSTM32L1xx( userfile );
			break;
		case 0x54:  // STM32L4
			returnNumber = DownloadFlashOfSTM32L4xx( userfile );
			break;
		case 0x55:
			returnNumber = DownloadFlashOfblueNRG_1( userfile );
			break;
		case 0x56:
			break;
		case 0x60:
			OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_NON_STRICT,&err);  // ��������˷�������������Ĺر��жϣ�������STM8��ʱ���õ������ᵼ�����سɹ�������������ã����»������������100ms��ʱ���ã���
			__disable_irq();
			returnNumber = DownloadFlashOfSTM8( userfile );
			{
					if( returnNumber == 0xFFBB && forDownloaderAgain == 1 )
					{
						forDownloaderAgain = 0;
						goto reDownloader;
					}
			}
			if( AutoDownloadEnable != ENABLE )
			{
				if( returnNumber != 0x55aa )
				{
					BeepCtrol(2,200);
				}
			}
			__enable_irq();
			break;
		case 0x61:
			break;
		case 0x62:
			break;
		default:
			break;
	}
	return returnNumber;  // ����ָʾ
}


