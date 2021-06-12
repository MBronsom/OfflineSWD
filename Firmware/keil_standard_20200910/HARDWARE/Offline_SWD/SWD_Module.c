/**
* @brief  用IO模拟SWD，这个注定是一个大工程！但是也是日后和利益点，越早越好
* @Author 
* @Data   2017.12.21
* \Attention 本次函数，因为要用到实际项目，其中的出错管理一定要特别好，务必保证稳定的前提下，增加速度，日后肯定得用SPI协议之类来套的，因为通过写LCD确实发现模拟的速度
						不敢恭维，比上硬件的SPI确实是不行的。
*/

#include "SWD_Module.h"

vu8 slowModeEnable = 0;  // 默认不使能。0：不使能。1：使能！
vu8 AutoDownloadEnable = 0;
void SWD_Delay( void )  
{
	for( vu32 i=0;i<=3;i++ );   // 用于方便移植操作
}
/**
 *  @B 初始化SWD引脚和需要的功能！
 *
 */

void SWDInit( void )
{
	{ // IO引脚初始化
		{ // 警告：使用了PB3/PB4，而这个是调试接口，默认是这个功能，所以需要去掉。
			RCC->APB2ENR.AFIOClockEnable_RW = ENABLE;     // 使能时钟
			AFIO->MAPR.SerialWireJTAGConfiguration_W = 2; // 只启用SWD引脚！
		}
		
		if( HARDWARE_VERSION == LCD_VERSION )
		{
			RCC->APB2ENR.IOPBClockEnable_RW = ENABLE;
			SWD_SWDIO_MODE_OUT;  // 输出模式
			SWD_SWDIO_PIN_OUT = 1;  // 输出为1
			
			GPIOB->CRL.InOutMode6_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
			GPIOB->CRL.PinConfig6_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL;  // RESET
			SWD_RESET_PIN = 1;   // 低电平复位，所以不复位
			{  // SWCLK引脚控制：因为本身含有上拉电阻，所以使用开漏输出到1。
				RCC->APB2ENR.IOPAClockEnable_RW = ENABLE;
				GPIOA->CRH.InOutMode8_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
				GPIOA->CRH.PinConfig8_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL;  // 根据发现：使用开漏引脚，也会使输出0的能力降低，造成通信距离太短，出现dpID和错误和过程中容易出错！
				SWD_SWCLK_PIN = 1;   // 默认为高电平
				
				GPIOA->CRL.InOutMode0_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
				GPIOA->CRL.PinConfig0_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_OPEN_DRAIN;  // 引脚开漏输出
				GPIOA->ODR.OutputData0_RW = 1;
				
				GPIOA->CRL.InOutMode1_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
				GPIOA->CRL.PinConfig1_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_OPEN_DRAIN;  // 引脚开漏输出
				GPIOA->ODR.OutputData1_RW = 1;
			}
		}
		else
		{
			RCC->APB2ENR.IOPAClockEnable_RW = ENABLE;
			SWD_SWDIO_MODE_OUT;  // 输出模式
			SWD_SWDIO_PIN_OUT = 1;  // 输出为1
			
			GPIOB->CRL.InOutMode6_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
			GPIOB->CRL.PinConfig6_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL;  // RESET
			SWD_RESET_PIN = 1;   // 低电平复位，所以不复位
			{ // 因为两个RESET引脚都用来复位，所以还需要PB1
				GPIOB->CRL.InOutMode1_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
				GPIOB->CRL.PinConfig1_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL;  // RESET
				GPIOB->ODR.OutputData1_RW = 1;
			}
			{  // SWCLK引脚控制：因为本身含有上拉电阻，所以使用开漏输出到1。
				RCC->APB2ENR.IOPAClockEnable_RW = ENABLE;
				GPIOA->CRL.InOutMode2_RW = GPIO_CR_INOUNTMODE_OUTPUT_50MHZ;
				GPIOA->CRL.PinConfig2_RW = GPIO_CR_PINCONFG_OUT_GENERAL_PURPOSE_PUSHPULL;  // 根据发现：使用开漏引脚，也会使输出0的能力降低，造成通信距离太短，出现dpID和错误和过程中容易出错！
				SWD_SWCLK_PIN = 1;   // 默认为高电平
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
 *  @B 暂时的目标是要求稳定！所以其中的延时比较多！
 *  @Warning 在转换方向的时候，需要注意，因为自己加了缓冲器,方向需要注意！
 *  结尾后:SWCLK保持为1
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
 *  @B 可以理解为TurnAround
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
 *  @B 可以理解为TurnAround
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
 *  @B 从AP或者DP寄存器里读出数据
 *     reg:显然只有4个(暂定的取值范围0-3)。  data为什么用指针，因为这样才能改变传递值，也就是指针指向的值
 *  结尾后:SWCLK保持为1
 */
u32 readReg( u8 APnDPReg,u8 reg, u32 *data )
{
	u8 i = 0;
	u8 cb = 0;  // 
	u8 parity;  // 校验值
	u8 b = 0;				// 用于读ACK的位
	u8 ack = 0; // ACK的值
	u8 ret = SWD_ERROR_OK;
	
	*data = 0;
	
	int _APnDPReg = (int) APnDPReg;
  int _read = (int) 1;  // 读请求值为1
	
	u8 A2 = reg & 0x01;
	u8 A3 = ( reg>>1 ) & 0x01;
	
	parity = ( _APnDPReg + _read + A2 + A3 ) & 0x01;
	
	SWD_SWDIO_MODE_OUT;   // 设置为输出模式
//	SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // 缓冲器设置为输出模式
	
	{  // 启动发送序列
		// 发送序列的问题：发送后，可以看出，其中SWCLK保持为1:
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
			SWD_SWDIO_MODE_IN;   // 设置为输入模式
//			SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_IN;  // 缓冲器设置为输入模式
		}
		SWCLK_CYCLE();
	}
	{  // 读ACK
		for( i=0;i<3;i++ )
		{
			b = READ_BIT( );
			ack |= b<<i;
		}
		  // 按照我的理解，这里不是应该有一个Trn的么？但是却没有
	}
	{ // 判断ACK位
		if( ack == ACK_OK )
		{
			for( i=0;i<32;i++)
			{
				b = READ_BIT( );
				*data |= b<<i;
				if(b)
					cb = !cb;  // cb之前已经初始化了
			}
			parity = READ_BIT();  // 最后再读一下校验位
			                      // 要使系统稳定的话，处理错误至关重要
			if( cb == parity )
			{
				ret = SWD_ERROR_OK;  // 系统正常
			}
			else
			{
				ret = SWD_ERROR_PARITY;  // 校验错误，检验错误就可能是位操作不对，当然这个概率是50%
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
			ret = SWD_ERROR_PROTOCOL;  //协议出错，这个要注意了
		}
	}
	
	{ // Turnaround
		SWCLK_CYCLE();
	}
	{ // 进入8个Idle状态，确保传输
		{
			SWD_SWDIO_MODE_OUT;   // 设置为输入模式
//			SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // 缓冲器设置为输入模式
		}
		for( i=0;i<8;i++ )
		{
			WRITE_BIT(0);
		}
	}
	return ret;    // 返回值
}


/**
 *  @B 这里有个要注意：可以忽视Ack，因为清除错误需要访问内容寄存器
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
	
	parity = ( _APnDPReg + _read +A2 + A3 )&0x1;  // 计算校验值
	
	SWD_SWDIO_MODE_OUT;   // 设置为输出模式
//	SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // 缓冲器设置为输出模式
	
	{  // 启动发送序列
		// 发送序列的问题：发送后，可以看出，其中SWCLK保持为1:
		WRITE_BIT( 1 );
		WRITE_BIT( _APnDPReg );
		WRITE_BIT( _read );
		WRITE_BIT( A2 );
		WRITE_BIT( A3 );
		WRITE_BIT( parity );
		WRITE_BIT( 0 );
		WRITE_BIT( 1 );   // SWDIO = 1, SWCLK = 0, SWCLK = 1 （这个由主机驱动为1的！手册说是不用主机驱动，但是由于上拉总线上会为1）
	}
	{
		{ // 发现一个特别奇葩的问题：就是如果把这一句，放到SWCLK_CYCLE();之后就会出现：ACK错误，比较费解！
			// 可能的原因：就是从机这个时候要驱动SWDIO，但是发现驱动不下来！！
			// 最好的解决办法：就是通过读DP寄存器找到这个问题！
			SWD_SWDIO_MODE_IN; 
//			SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_IN;
		}
		SWCLK_CYCLE();
	}
	{
		{ 
			
			// 读ACK
			for( i=0;i<3;i++ )
			{
				b = READ_BIT( );
				ack |= b<<i;
			}
				// 按照我的理解，这里不是应该有一个Trn的么？但是却没有，（最后发现读是没有的人，但是写是有的！写的在下面）
		}
	}
	{ // 
		if( (ack == ACK_OK) || ignoreAck )
		{
			
			{ // 设置为输出
				SWD_SWDIO_MODE_OUT;   // 设置为输出模式
//				SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // 缓冲器设置为输出模式
			}
			SWCLK_CYCLE();  // 写的时候要用Trn一次。但是读的时候没有在代码里面看到。
			parity = 0;    // 这个函数有问题，当我改parity的时候，出现可以读状态，说明给器件的值存在检验错误！！这个是不应该的。这个要检查！
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
			ret = SWD_ERROR_PROTOCOL;  //协议出错，这个要注意了
		}
		
		{ // 进入8个Idle状态，确保传输，这个可以优化！
			{  // 这个其实是多余的，一直是输出的，所以我都考虑这个源代码的性能了。
				SWD_SWDIO_MODE_OUT;   // 设置为输入模式
//				SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // 缓冲器设置为输入模式
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
 *  @B 上电执行JTAG转SWD的序列函数！
 *
 */
void JTAG_To_SWD_Sequence( void )
{
  int i;
  int b;
  
	SWD_SWDIO_MODE_OUT;   // 设置为输出模式
//	SWD_SWDIO_DIR_CTR2 = BUFFER_IC_DIR_OUT;  // 缓冲器设置为输出模式
	{
		SWD_SWDIO_PIN_OUT = 1;   // 先输出为1
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
	{ // 执行16个空闲周期
		SWD_SWDIO_PIN_OUT = 0;
		for( i=0;i<16;i++ )
		{
			SWCLK_CYCLE();
		}
		SWD_SWDIO_PIN_OUT = 1;  // 实际中，因为少了，这一句导致只能下载F1，F4/F2总是出现问题。
	}
		
}


/**
 *  @B 写AP寄存器，在当前选择的APBANK。
 *
 */


void writeAP( u8 reg,u32 data)
{
	u8 forFault = 0;
	forWait:
	{
		u8 swdStatus;
		u8 retry = 1;  // 这个是重试次数，但是日后应该做下载器，则这个变量是一定要提取出来的
		do
		{
			swdStatus = writeReg(1,reg,data,FALSE);
			retry--;
		}while( (swdStatus == SWD_ERROR_WAIT) && retry>0 );
		if( swdStatus != SWD_ERROR_OK )
		{
			if( swdStatus == SWD_ERROR_WAIT )
			{
				goto forWait;  // 在测试F2的时候，出现了不定时的这个问题，所以才用这个方法！
			}
			else if( swdStatus == SWD_ERROR_FAULT )
			{
				if( forFault == 0 ) // 重试次数超出
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
				RAISE( swdStatus );  // 根据查询源码，这个函数的操作相对比较麻烦，需要自己日后重新定义！
			}
		}
	}
}

/**
 *  @B 写DP寄存器
 *
 */
void writeDP( u8 reg,u32 data )
{
	u8 forFault = 0;
	forWait:
	{
		u8 swdStatus;
		u8 retry = 10;  // 重试次数
		do
		{
			swdStatus = writeReg(0,reg,data,FALSE);
			retry--;
		}while( (swdStatus == SWD_ERROR_WAIT) && retry>0 );
		if( swdStatus != SWD_ERROR_OK )
		{
			if( swdStatus == SWD_ERROR_WAIT )
				{
					goto forWait;  // 在测试F2的时候，出现了不定时的这个问题，所以才用这个方法！
				}
				else if( swdStatus == SWD_ERROR_FAULT )
				{
					if( forFault == 0 ) // 重试次数超出
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
 *  @B 写DP寄存器但是忽略ACK位，这个显然是出错之后的操作！
 *
 */
void writeDPIgnoreAck( u8 reg,u32 data )
{
	u8 forFault = 0;
	forWait:
	{
		u8 swdStatus;
		u8 retry = 10;  // 重试次数
		do
		{
			swdStatus = writeReg(0,reg,data,TRUE);
			retry--;
		}while( (swdStatus == SWD_ERROR_WAIT) && retry>0 );
		if( swdStatus != SWD_ERROR_OK )
		{
			if( swdStatus == SWD_ERROR_WAIT)
				{
					goto forWait;  // 在测试F2的时候，出现了不定时的这个问题，所以才用这个方法！
				}
				else if( swdStatus == SWD_ERROR_FAULT )
				{
					if( forFault == 0 ) // 重试次数超出
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
					RAISE( swdStatus );  // 出错处理特别重要！
				}
		}
	}
}

/**
 *  @B 读AP
 *
 */
void readAP( u8 reg,u32 *data )
{
	u8 forFault = 0;
	forWait:
	{
		u8 swdStatus;
		u8 retry = 10;  // 重试次数
		do
		{
			swdStatus = readReg( 1,reg,data );
			retry--;
		}while( (swdStatus == SWD_ERROR_WAIT) && retry>0 );
		if( swdStatus != SWD_ERROR_OK )
		{
			if( swdStatus == SWD_ERROR_WAIT)
				{
					goto forWait;  // 在测试F2的时候，出现了不定时的这个问题，所以才用这个方法！
				}
				else if( swdStatus == SWD_ERROR_FAULT )
				{
					if( forFault == 0 ) // 重试次数超出
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
					RAISE( swdStatus );  // 出错处理特别重要！
				}
		}
	}
}

/**
 *  @B 读DP
 *
 */
void readDP( u8 reg,u32 * data )
{
	u8 forFault = 0;
	forWait:
	{
		u8 swdStatus;
		u8 retry = 10;  // 重试次数
		do
		{
			swdStatus = readReg( 0,reg,data );
			retry--;
		}while( (swdStatus == SWD_ERROR_WAIT) && retry>0 );
		if( swdStatus != SWD_ERROR_OK )
		{
			if( swdStatus == SWD_ERROR_WAIT)
				{
					goto forWait;  // 在测试F2的时候，出现了不定时的这个问题，所以才用这个方法！
				}
				else if( swdStatus == SWD_ERROR_FAULT )
				{
					if( forFault == 0 ) // 重试次数超出
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
					RAISE( swdStatus );  // 出错处理特别重要！
				}
		}
	}
}


/**
 *  @B 初始化DP，先发送一个转换序列，然后读出IDCode的值。
 *
 */
u32 initDp( void )
{
	u32 dpId;
	JTAG_To_SWD_Sequence(  );
	readDP(DP_IDCODE,&dpId);  // 
	
	//Debug power up request，这个应该是打开调试组件的电源，方便调试！
	writeDP( DP_CTRL, DP_CTRL_CSYSPWRUPREQ | DP_CTRL_CDBGPWRUPREQ );
	
	// Wait until we receive powerup ACK 
	int retry = 300;
	u32 status;
	while( retry>0 )
	{
		readDP(DP_CTRL,&status);
		// 下面的这个是什么意思？
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
//    RAISE( SWD_ERROR_DEBUG_POWER );  // 这个出错不好管理！
		RAISE( status );
  }
  
  /* Select first AP bank */
  writeDP( DP_SELECT, 0x00 );
  
  return dpId;  // 应该是1BA01477(F4读出的值为：2BA01477)  F2测试也是：2BA01477
}



/**
 *  @B 这个可能是针对EMF32单片机的，但是有一定的参考价值！这也能读出来东西，但是不知道是什么含义！
 */
u32 readApID( void )
{
	u32 apId;
	/* Select last AP bank */
  writeDP( DP_SELECT, 0xf0 );  // 写DP寄存器中的APBBANKSEL位为0xF
//	writeDP( DP_SELECT, 0x08000003 );
  
  /* Dummy read AP ID */
  readAP( AP_IDR, &apId );
  
  /* Read AP ID */
  readDP( DP_RDBUFF, &apId );
  
  /* Select first AP bank again */
  writeDP( DP_SELECT, 0x00 );
  
  return apId; // EMF手册中说明这个值应该是：0x24770011（F4读出的值是这个！），但是实际读出来是0x14770011（F103）。不知道这是什么含义！
}

/**
 *  @B 不明觉厉
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



/* 从下面开始，就是移植的另外一个程序--在SRAM中编程 ，具有不错的参考价值 */
/**
 *  @B 对于程序，务必要停止运行后，再写FLash操作！所以这个函数存在
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

/*以上，就是移植的另外一个程序--在SRAM中编程 ，具有不错的参考价值 */

/* 从下面开始，是自己的测试程序 */
u32 readMemoryFormStm32( void )
{
	u32 apId = 0;
	vu32 address = 0x08000000;
	/* Select last AP bank */
//	while(1)
	{  // 按照下面的程序测试，速度大约是：4MHZ左右(3.7MHZ)测试一次！还是比较给力的。
		writeDP( DP_SELECT, 0x00 );  // 访问寄存器
	//	writeDP( DP_SELECT, 0x08000003 );
		
		writeAP( AP_CSW,0x2);   // 长度为32bit
		
		writeAP( AP_TAR,address);   // 写目标地址
		readAP( AP_DRW,&apId);
		
		/* Read AP ID */
		readDP( DP_RDBUFF, &apId );
		address++;
  }
  
  return apId;
}

/**
 *  @B 测试
 *
 */

void testFlashChearAndWrite( void )
{
	{	// 清空Flash位
		
	}
	{ // 写Flash
		
	}
}


#define CMX_AIRCR 0xE000ED0C
#define RUN_CMD_CRJ 0x05FA0007  // 这个是从：DAP中提取出来的，不知道行不行！！

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
	
	
	writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
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
	{ // 手册给的建议是：使能一个断点！！
	}
}

/**
 *  @B 测试成功，可以启动程序！但是要注意CSW的值，需要设置为：0x23000002（暂时不明白含义，这个要注意！日后再做！）
 *
 */
void runTarget(void)
{
	uint32_t i;
	writeAP( AP_CSW,0x23000002); 
	
	writeAP(AP_TAR, CM3_DEMCR);
  writeAP(AP_DRW, 0x01000400);  // 避免复位停在Reset！
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
 *  @B 清空STM32 Flash操作
 *
 */
void clearAllFlash( void )
{
	OS_ERR err;
	writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
	
  writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
  writeAP(AP_DRW, FLASH_KEY1);
	
	writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
  writeAP(AP_DRW, FLASH_KEY2);
	
	{ // 清除错误
			writeAP(AP_TAR, (u32)&FLASH->SR.All ) ;
			writeAP(AP_DRW, _0b00110100);
	}
	
	writeAP( AP_CSW,0x00000002);   // 读
	u32 readWord0 = 0;
	do
	{
		writeAP(AP_TAR, (u32)&FLASH->SR.All);
		readAP(AP_DRW,&readWord0);
		readDP(DP_RDBUFF,&readWord0);
	}while(readWord0&0x1); // 当为1的时候说明没有操作完成！
	
	writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
	writeAP( AP_TAR, (u32)&FLASH->CR.All );
  writeAP( AP_DRW, _0b00000100 );  // 注意：擦除的时候要分开写。
	
	writeAP( AP_TAR, (u32)&FLASH->CR.All );
  writeAP( AP_DRW, _0b01000100 );  // 先写清除全片，再写开始位！！
	
	writeAP( AP_CSW,0x00000002);   // 读
	u32 readWord1 = 0;
	do
	{
		writeAP(AP_TAR, (u32)&FLASH->SR.All);
		readAP(AP_DRW,&readWord1);
		readDP(DP_RDBUFF,&readWord1);
	}while(readWord1&0x1); // 当为1的时候说明没有操作完成！
	{ // 根据客户反应，全片擦除有可能存在没有完成的地方，所以本次严格的按照操作来操作！！
		// 
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while( ( readWord1&_0b00100000 ) != _0b00100000);
	}
	{ // 清除错误
			writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
			writeAP(AP_TAR, (u32)&FLASH->SR.All);
			  writeAP(AP_DRW, _0b00110100);
	}
	OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err); 
}


/**
 *  @B 写Flash程序操作！测试！
 *
 */

void writeFlash( void )
{
	
}


/**
 *  @B F0系列的保护操作选项，注意：F0系列的FLASH编程也是16bit一次编程这个和F1的操作是一样的。
    @warning 实际中发现F0的寄存器分配和F1分配是一样的，所以选择用F1的寄存器进行操作。
 *  
 */
void stm32f0Protection( u8 readProtectionEnable,u8 writeProtectionEnable )
{
	u32 readWord1 = 0;
//	if( (readProtectionEnable == 1) || (writeProtectionEnable == 1 ))  // 
//	{
		{  // 解锁Flash操作
			writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
			
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
			}while(readWord1&0x1); // 当为1的时候说明没有操作完成！
		}		
		{ // 解锁OptionsByte操作
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY1);
			
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY2);
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00100000 + 0x200 );  // 清除选项字节，注意：此时读保护依然在，写保护被清除，用户字节被清除
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b01100000 + 0x200 );  // 开始操作
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // 忙时卡住
		}
		{ // 先不要置位读保护
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 );  // 恢复现场
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00010000 + 0x200 );  // 选项字节编程使能！
		}
		{
			if( readProtectionEnable == ENABLE )  // 默认状态下就是读保护使能的！（因为清除了！）
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0x00);  // 0x00也是读保护
			}
			else
			{
				writeAP( AP_CSW,0x23000001 ); // 长度为16bit,写（操作Flash必须用u16形式，否则会出问题！）
				writeAP( AP_TAR, (u32)&OB->RDP );
				if( ((u32)&OB->RDP)%4 != 0 )
				{
					writeAP( AP_DRW, 0xAA<<16 );  // 清除读保护，当值为0xAA时，关读保护！
				}
				else
				{
					writeAP( AP_DRW, 0xAA);  // 清除读保护
				}
			}
			if( writeProtectionEnable == ENABLE )
			{
				writeAP( AP_CSW,0x23000001 ); // 长度为16bit,写（操作Flash必须用u16形式，否则会出问题！）
				writeAP( AP_TAR, (u32)&OB->WRP0 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP1 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP2 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP3 );
				writeAP( AP_DRW, 0x00 );
				
			}
			else // 之前已经清除了，所以不需要管！
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0xFFFFFFFF);  // 清除读保护
			}
			
			writeAP( AP_CSW,0x23000002 ); // 恢复为32位
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // 忙时卡住
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 );  // 恢复现场
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b10000000);  	// 锁定操作
		}
}
/**
 *  @B 保护的使能
 *  警告：操作选项字节也必须通过u16型的访问，否则会出错！不能成功操作！
 *  
 */

void stm32f1Protection( u8 readProtectionEnable,u8 writeProtectionEnable )
{
	u32 readWord1 = 0;
//	if( (readProtectionEnable == 1) || (writeProtectionEnable == 1 ))  // 
//	{
		{  // 解锁Flash操作
			writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
			
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
			}while(readWord1&0x1); // 当为1的时候说明没有操作完成！
		}		
		{ // 解锁OptionsByte操作
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY1);
			
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY2);
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00100000 + 0x200 );  // 清除选项字节，注意：此时读保护依然在，写保护被清除，用户字节被清除
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b01100000 + 0x200 );  // 开始操作
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // 忙时卡住
		}
		{ // 先不要置位读保护
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 );  // 恢复现场
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00010000 + 0x200 );  // 选项字节编程使能！
		}
		{
			if( readProtectionEnable == ENABLE )  // 默认状态下就是读保护使能的！
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0x00);  // 0x00也是读保护
			}
			else
			{
				writeAP( AP_CSW,0x23000001 ); // 长度为16bit,写（操作Flash必须用u16形式，否则会出问题！）
				writeAP( AP_TAR, (u32)&OB->RDP );
				if( ((u32)&OB->RDP)%4 != 0 )
				{
					writeAP( AP_DRW, RDP_Key<<16 );  // 清除读保护
				}
				else
				{
					writeAP( AP_DRW, RDP_Key);  // 清除读保护
				}
			}
			if( writeProtectionEnable == ENABLE )
			{
				writeAP( AP_CSW,0x23000001 ); // 长度为16bit,写（操作Flash必须用u16形式，否则会出问题！）
				writeAP( AP_TAR, (u32)&OB->WRP0 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP1 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP2 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP3 );
				writeAP( AP_DRW, 0x00 );
				
			}
			else // 之前已经清除了，所以不需要管！
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0xFFFFFFFF);  // 清除读保护
			}
			
			writeAP( AP_CSW,0x23000002 ); // 恢复为32位
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // 忙时卡住
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 + 0x2000);  // 恢复现场，最后一个的0x2000，对于GD32是：强制重新加载选项字节！			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b10000000 + 0x2000);  	// 锁定操作
		}
}


void stm32f3Protection( u8 readProtectionEnable,u8 writeProtectionEnable )
{
	u32 readWord1 = 0;
//	if( (readProtectionEnable == 1) || (writeProtectionEnable == 1 ))  // 
//	{
		{  // 解锁Flash操作
			writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
			
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
			}while(readWord1&0x1); // 当为1的时候说明没有操作完成！
		}		
		{ // 解锁OptionsByte操作
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY1);
			
			writeAP(AP_TAR, (u32)&FLASH->OPTKEYR.OptionByteKey_W);
			writeAP(AP_DRW, FLASH_KEY2);
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00100000 + 0x200 );  // 清除选项字节，注意：此时读保护依然在，写保护被清除，用户字节被清除
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b01100000 + 0x200 );  // 开始操作
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // 忙时卡住
		}
		{ // 先不要置位读保护
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 );  // 恢复现场
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00010000 + 0x200 );  // 选项字节编程使能！
		}
		{
			if( readProtectionEnable == ENABLE )  // 默认状态下就是读保护使能的！
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0x00);  // 0x00也是读保护
			}
			else
			{
				writeAP( AP_CSW,0x23000001 ); // 长度为16bit,写（操作Flash必须用u16形式，否则会出问题！）
				writeAP( AP_TAR, (u32)&OB->RDP );
				if( ((u32)&OB->RDP)%4 != 0 )
				{
					writeAP( AP_DRW, 0xAA<<16 );  // 清除读保护
				}
				else
				{
					writeAP( AP_DRW, 0xAA);  // 清除读保护
				}
			}
			if( writeProtectionEnable == ENABLE )
			{
				writeAP( AP_CSW,0x23000001 ); // 长度为16bit,写（操作Flash必须用u16形式，否则会出问题！）
				writeAP( AP_TAR, (u32)&OB->WRP0 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP1 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP2 );
				writeAP( AP_DRW, 0x00 );
				
				writeAP( AP_TAR, (u32)&OB->WRP3 );
				writeAP( AP_DRW, 0x00 );
				
			}
			else // 之前已经清除了，所以不需要管！
			{
//				writeAP(AP_TAR, (u32)&OB->RDP);
//				writeAP(AP_DRW, 0xFFFFFFFF);  // 清除读保护
			}
			
			writeAP( AP_CSW,0x23000002 ); // 恢复为32位
			
			do
			{
				writeAP(AP_TAR, (u32)&FLASH->SR.All);
				readAP(AP_DRW,&readWord1);
				readDP(DP_RDBUFF,&readWord1);
			}while(readWord1&0x1); // 忙时卡住
			
		}
		{
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b00000000 + 0x200 );  // 恢复现场
			
			writeAP(AP_TAR, (u32)&FLASH->CR.All);
			writeAP(AP_DRW, _0b10000000);  	// 锁定操作
		}
}

/**
 *  @B F2系列的读写保护操作，注意其中的操作是直接从F4移植过来的，需要充分的验证！
 *  
 */

u32 stm32f2Protection( u8 readProtectionEnable,u8 writeProtectionEnable )
{
//	u32 readWord1 = 0;
//	
//	u32 protectValue0 = 0; // 寄存器OPTCR的值
////	u32 protectValue1 = 0; // 寄存器OPTCR1的值
//	
////	u32 F4FlashMemorySize = 0;  // 在操作擦除操作和保护操作时，必须要知道其大小，否则会出现没有定义的值！
////		{ // 检查Flash的大小。
////			u32 apId = 0;
////			writeDP( DP_SELECT, 0x00 );  // 访问寄存器
////			writeAP( AP_CSW,0x2 );   // 长度为32bit，同时地址自动增加
////			writeAP( AP_TAR,(u32)(0x1FFF7A22));   // 写目标地址
////			readAP( AP_DRW,&apId);  // 第一次数据，不要！
////			readDP( DP_RDBUFF, &apId );
////			F4FlashMemorySize = apId>>16;
////		}
//		writeDP( DP_ABORT,0x1F);
//	
//	if( readProtectionEnable == ENABLE )  // 使能读保护
//	{
//		if( writeProtectionEnable == ENABLE) // 使能写保护
//		{
//			protectValue0 = 0x0000FFED - 1;  // 使能写保护，RDP用0xFF，使能一级保护，
////			protectValue1 = 0x00000000;  // 使能写保护
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
//		else  // 全部都不使能
//		{
//			protectValue0 = 0x0FFFAAED - 1;
////			protectValue1 = 0x0FFF0000;
//		}
//	}
//	
//	{
//			{  // 解锁Flash操作
//				writeAP( AP_CSW,0x23000002);   // 32位写访问
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0x45670123 );
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0xCDEF89AB );
//				
//				writeAP( AP_CSW,0x00000002);   // 读
//				u32 readWord0 = 0;
//				do
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//					readAP(AP_DRW,&readWord0);
//					readDP(DP_RDBUFF,&readWord0);
//				}while((readWord0>>16)&0x1); // 当为1的时候说明没有操作完成！
//			}
//			{  // 解锁选项字节Flash操作
//				writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
//				
//				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
//				writeAP(AP_DRW, 0x08192A3B);
//				
//				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
//				writeAP(AP_DRW, 0x4C5D6E7F);
//			}
//			{  // 操作前要确认没有Flash进行操作
//				do
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//					readAP(AP_DRW,&readWord1);
//					readDP(DP_RDBUFF,&readWord1);
//				}while( (readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
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
//				writeAP(AP_DRW, protectValue0 + 2 );  // 开始操作
//				
//				do
//					{
//						writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//						readAP(AP_DRW,&readWord1);
//						readDP(DP_RDBUFF,&readWord1);
//					}while( (readWord1>>16)&0x1); // 忙时卡住
//			}
//	}

	u32 readWord1 = 0;
	
	u32 protectValue0 = 0; // 寄存器OPTCR的值
//	u32 protectValue1 = 0; // 寄存器OPTCR1的值
	
//	u32 F4FlashMemorySize = 0;  // 在操作擦除操作和保护操作时，必须要知道其大小，否则会出现没有定义的值！
//		{ // 检查Flash的大小。
//			u32 apId = 0;
//			writeDP( DP_SELECT, 0x00 );  // 访问寄存器
//			writeAP( AP_CSW,0x2 );   // 长度为32bit，同时地址自动增加
//			writeAP( AP_TAR,(u32)(0x1FFF7A22));   // 写目标地址
//			readAP( AP_DRW,&apId);  // 第一次数据，不要！
//			readDP( DP_RDBUFF, &apId );
//			F4FlashMemorySize = apId>>16;
//		}
		writeDP( DP_ABORT,0x1F);
	
	if( readProtectionEnable == ENABLE )  // 使能读保护
	{
		if( writeProtectionEnable == ENABLE) // 使能写保护
		{
			protectValue0 = 0x0000FFED - 1;  // 使能写保护，RDP用0xFF，使能一级保护，
//			protectValue1 = 0x00000000;  // 使能写保护
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
		else  // 全部都不使能
		{
			protectValue0 = 0x0FFFAAED - 1;
//			protectValue1 = 0x0FFF0000;
		}
	}
	
	{
			{  // 解锁Flash操作  --- 警告：不能解锁Flash，否则会出现错误，而且之前已经解锁Flash过了，这是实测的数据，所以问题的具体原因，日后再解决！！估计是程序操作中有操作Flash的地方！！
//				writeAP( AP_CSW,0x23000002);   // 32位写访问
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0x45670123 );
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0xCDEF89AB );
				
//				writeAP( AP_CSW,0x00000002);   // 读
//				u32 readWord0 = 0;
//				do
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//					readAP(AP_DRW,&readWord0);
//					readDP(DP_RDBUFF,&readWord0);
//				}while((readWord0>>16)&0x1); // 当为1的时候说明没有操作完成！
			}
			{  // 解锁选项字节Flash操作
				writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
				writeAP(AP_DRW, 0x08192A3B);
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
				writeAP(AP_DRW, 0x4C5D6E7F);
			}
			{  // 操作前要确认没有Flash进行操作
				writeAP( AP_CSW,0x00000002);   // 读
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord1);
					readDP(DP_RDBUFF,&readWord1);
				}while( (readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
			}	
			
			
			
			{
				writeAP( AP_CSW,0x23000002 ); // 写
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
				writeAP(AP_DRW, protectValue0 );
				
//				if( F4FlashMemorySize >0x400 )
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR1);
//					writeAP(AP_DRW, protectValue1 );
//				}
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR );
				writeAP(AP_DRW, protectValue0 + 2 );  // 开始操作
				
				writeAP( AP_CSW,0x00000002);   // 读
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
					}while( (readWord1>>16)&0x1); // 忙时卡住
					
					{ // 检测结果，这个很重要，因为这个涉及到保护，而且选项字节是有加载时间的。有时候需要断电重启。这个比较关键
						
						writeAP( AP_CSW,0x00000002);   // 读
						writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
						readAP(AP_DRW,&readWord1);
						readDP(DP_RDBUFF,&readWord1);
						
						if( ( readWord1 & 0xFFFFF00 ) == ( protectValue0 & 0xFFFFF00 ) )  // 把写的值和读的值做比较！
						{
							return 0x55AA;
						}
						else
						{
							if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12+12*10,12,RED,"  选项字节检查错误   " );
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
//							if( ( readWord1 & 0xFFF0000 ) == ( protectValue1 & 0xFFF0000 ) )  // 把写的值和读的值做比较！
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
 *  @B F4系列的读写保护操作
 *  注意：F4系列的保护，可以直接操作寄存器。应该可以32位访问的。
 *  
 */


u32 stm32f4Protection( u8 readProtectionEnable,u8 writeProtectionEnable )
{
	u32 readWord1 = 0;
	
	u32 protectValue0 = 0; // 寄存器OPTCR的值
	u32 protectValue1 = 0; // 寄存器OPTCR1的值
	
	u32 F4FlashMemorySize = 0;  // 在操作擦除操作和保护操作时，必须要知道其大小，否则会出现没有定义的值！
		{ // 检查Flash的大小。
			u32 apId = 0;
			writeDP( DP_SELECT, 0x00 );  // 访问寄存器
			writeAP( AP_CSW,0x2 );   // 长度为32bit，同时地址自动增加
			writeAP( AP_TAR,(u32)(0x1FFF7A22));   // 写目标地址
			readAP( AP_DRW,&apId);  // 第一次数据，不要！
			readDP( DP_RDBUFF, &apId );
			F4FlashMemorySize = apId>>16;
		}
		writeDP( DP_ABORT,0x1F);
	
	if( readProtectionEnable == ENABLE )  // 使能读保护
	{
		if( writeProtectionEnable == ENABLE) // 使能写保护
		{
			protectValue0 = 0x0000FFED - 1;  // 使能写保护，RDP用0xFF，使能一级保护，
			protectValue1 = 0x00000000;  // 使能写保护
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
		else  // 全部都不使能
		{
			protectValue0 = 0x0FFFAAED - 1;
			protectValue1 = 0x0FFF0000;
		}
	}
	
	{
			{  // 解锁Flash操作  --- 警告：不能解锁Flash，否则会出现错误，而且之前已经解锁Flash过了，这是实测的数据，所以问题的具体原因，日后再解决！！估计是程序操作中有操作Flash的地方！！
//				writeAP( AP_CSW,0x23000002);   // 32位写访问
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0x45670123 );
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
//				writeAP( AP_DRW, 0xCDEF89AB );
				
//				writeAP( AP_CSW,0x00000002);   // 读
//				u32 readWord0 = 0;
//				do
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//					readAP(AP_DRW,&readWord0);
//					readDP(DP_RDBUFF,&readWord0);
//				}while((readWord0>>16)&0x1); // 当为1的时候说明没有操作完成！
			}
			{  // 解锁选项字节Flash操作
				writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
				writeAP(AP_DRW, 0x08192A3B);
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTKEYR);
				writeAP(AP_DRW, 0x4C5D6E7F);
			}
			{  // 操作前要确认没有Flash进行操作
				writeAP( AP_CSW,0x00000002);   // 读
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord1);
					readDP(DP_RDBUFF,&readWord1);
				}while( (readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
			}	
			
			
			
			{
				writeAP( AP_CSW,0x23000002 ); // 写
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
				writeAP(AP_DRW, protectValue0 );
				
				if( F4FlashMemorySize >0x400 )
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR1);
					writeAP(AP_DRW, protectValue1 );
				}
				
				writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR );
				writeAP(AP_DRW, protectValue0 + 2 );  // 开始操作
				
				writeAP( AP_CSW,0x00000002);   // 读
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
					}while( (readWord1>>16)&0x1); // 忙时卡住
					
					{ // 检测结果，这个很重要，因为这个涉及到保护，而且选项字节是有加载时间的。有时候需要断电重启。这个比较关键
						
						writeAP( AP_CSW,0x00000002);   // 读
						writeAP(AP_TAR, (u32)&F4_FLASH->OPTCR);
						readAP(AP_DRW,&readWord1);
						readDP(DP_RDBUFF,&readWord1);
						
						if( ( readWord1 & 0xFFFFF00 ) == ( protectValue0 & 0xFFFFF00 ) )  // 把写的值和读的值做比较！
						{
							return 0x55AA;
						}
						else
						{
							if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12+12*10,12,RED,"  选项字节检查错误   " );
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
							if( ( readWord1 & 0xFFF0000 ) == ( protectValue1 & 0xFFF0000 ) )  // 把写的值和读的值做比较！
							{
								return 0x55AA;
							}
							else
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12+12*10,12,RED,"  选项字节检查错误   " );
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
 *  @B F0系列的下载函数
 *  
 */

u32 DownloadFlashOfSTM32F0xx( _FileInformation userfile )
{
	u32 returnNumber = 0;
	{
		{ // 
				{ // 检查保护区！
							OS_ERR err;
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // 访问寄存器
							writeAP( AP_CSW,0x2 );   // 长度为32bit，同时地址自动增加
							writeAP( AP_TAR,(u32)&OB->RDP);   // 写目标地址
							readAP( AP_DRW,&apId);  // 第一次数据，不要！
							readDP( DP_RDBUFF, &apId );
							if( apId == 0xFFFF55AA ) // 如果能操作到这一步表明没有设置读保护
							{												// 对于F0系列，发现读出的值为：0xFFFF5AA5，也就是：使能了读保护！操作的器件为Jlink，使用的J-Flash中的加密！
								{ // 检查写保护
									writeAP( AP_TAR,(u32)&OB->WRP0);
									readAP( AP_DRW,&apId);
									readDP( DP_RDBUFF, &apId );
									
									if( apId != 0xFFFFFFFF)  // 说明存在写保护
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      发现写保护      ");
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
									if( apId != 0xFFFFFFFF ) // 说明存在写保护
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      发现读保护      ");
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
							{ // 设置了读保护，要清除！
												if( L_ENGLISH_VER != 0x55AA )
												{
													LCD_ShowString(0,12*10,12,RED, "      发现读保护      ");
												}
												else
												{
													LCD_ShowString(0,12*10,12,RED, "      read protect    ");
												}
								dealTmp0:
												if( L_ENGLISH_VER != 0x55AA )
												{
														LCD_ShowString(0,12*11,12,RED, "    清除中... 2S     ");
												}
												else
												{
													LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 2S    ");
												}
								{ // 清除SWD的错误！
									writeDP( DP_ABORT,0x1F);
									stm32f0Protection( DISABLE,DISABLE );
								}
								OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    清除中... 1S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 1S    ");
								}
								OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    清除中... 0S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 0S    ");
								}
								
								LCD_ShowString(0,12*10,12,RED, "  Protection Cleared ");
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    请断电重新下载   ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "DownloadAgainWithoutPower");
								}
								{  // 重新下载选项！
										{
											writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
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
			
			{ // 清除Flash
				if( userfile.ChipEraseWay == 00 )  // 全擦除
				{
					clearAllFlash(  );  // 这个是清除全部Flash的操作
				}
				else	// 块擦除，分为小容量大容量和大容量，但是大容量也可以用1KB的来操作，这样可以简化，但是浪费时间，第一个版本不用太在意！
							// 除此之外，还有滚码所在的地址！
				{
					u32 startAddressTmp0 = 0x08000000; // 起始页所在地址
					u32 endAddressTmp1 = 0x08000000;   // 结束页所在地址
					u32 startRollingAddressTmp3 = 0x08000000;
					u32 endRollingAddressTmp4 = 0x08000000;
					for( u32 tmp5 = 0;tmp5<1024;tmp5++ ) // 1024：最大为1M大小！
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
						if( (userfile.RollingCodeStartAddress + 32 ) < ( 0x08000000 + 0x400 * tmp8 ) )  // 这个的SIZE只有32
						{
							endRollingAddressTmp4 = 0x08000000 + 0x400 * (tmp8 - 1);
							break;
						}
					}
					{ // 用SWD清除Flash操作
						writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
	
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY1);
						
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY2);
						
						writeAP( AP_CSW,0x00000002);   // 读
						u32 readWord0 = 0;
						do
						{
							writeAP(AP_TAR, (u32)&FLASH->SR.All);
							readAP(AP_DRW,&readWord0);
							readDP(DP_RDBUFF,&readWord0);
						}while(readWord0&0x1); // 当为1的时候说明没有操作完成！
						
						for( u32 addressTmp9 = startAddressTmp0; addressTmp9<= endAddressTmp1; addressTmp9 += 0x400 )  // 清除程序区
						{
							writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000010 );  // 页擦除使能
							
							writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
							writeAP( AP_DRW, addressTmp9 );  // 写入地址
							
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b01000010 );  // 开始
							
							writeAP( AP_CSW,0x00000002);   // 读
							u32 readWord1 = 0;
							do
							{
								writeAP(AP_TAR, (u32)&FLASH->SR.All);
								readAP(AP_DRW,&readWord1);
								readDP(DP_RDBUFF,&readWord1);
							}while(readWord1&0x1); // 当为1的时候说明没有操作完成！
						}
						{ // 恢复现场
							writeAP( AP_CSW,0x23000002);  
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000000 );
						}
						if( userfile.RollingCodeFunction == 0x55 ) // 使能滚码操作，只有当滚码操作使能的时候，再操作，防止访问了错误的地址！
						{
							for( u32 addressTmp10 = startRollingAddressTmp3; addressTmp10<= endRollingAddressTmp4; addressTmp10 += 0x400 )  // 清除滚码区
							{
								writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000010 );  // 页擦除使能
								
								writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
								writeAP( AP_DRW, addressTmp10 );  // 写入地址
								
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b01000010 );  // 开始
								
								writeAP( AP_CSW,0x00000002);   // 读
								u32 readWord1 = 0;
								do
								{
									writeAP(AP_TAR, (u32)&FLASH->SR.All);
									readAP(AP_DRW,&readWord1);
									readDP(DP_RDBUFF,&readWord1);
								}while(readWord1&0x1); // 当为1的时候说明没有操作完成！
							}
							{ // 恢复现场
								writeAP( AP_CSW,0x23000002);  
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000000 );
							}
						}
						{ // 锁定Flash
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b10000000 );
						}
					}
				}
			}
			{ // 写Flash，同时要写滚码操作区！
				
				{  // 解锁Flash操作
					writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY1);
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY2);
				}
				{ // 使能编辑位
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, 0x1);  // 使能编程操作！
				}
				
				{ // 写程序区
					u32 HowMany4K = 0;
					if( userfile.ProgramSize % 0x1000 != 0 )
						HowMany4K = userfile.ProgramSize/0x1000 + 1;
					else
						HowMany4K = userfile.ProgramSize/0x1000;
					
					for( u32 tmp11 = 0;tmp11<HowMany4K; tmp11++ )  // 写Flash的操作，每次操作4K字节
					{
						u32 Address = userfile.ProgramStartAddress + tmp11*0x1000;  // 起始操作地址
						W25QXX_Read( W25Q4KBuf.databuf4K , userfile.ProgramSaveAtW25QAddress + tmp11*0x1000, 0x1000); // 4k大小在16进行下，就是0x1000
						
						writeAP( AP_CSW,0x23000001 | 0x10 ); // 长度为16bit,写，且地址自动增加！
						writeAP( AP_TAR, Address );  // 地址！
						for( u32 tmp12 = 0;tmp12 <0x1000; tmp12 += 2 ) // 写入4K
						{
							{ // 实测，可以把下面的给省略掉，这样可以提升速度一个测试是：从56S - 36S！因为写单个可能根本不需要这样！但是这个暂时保留，日后防止速度上来后，影响速度！
//								writeAP( AP_CSW,0x00000002 );   // 读
//								do
//								{
//									writeAP(AP_TAR, (u32)&FLASH->SR.All);
//									readAP(AP_DRW,&readWord0);
//									readDP(DP_RDBUFF,&readWord0);
//								}while(readWord0&0x1); // 当为1的时候说明没有操作完成！
							}
							// 下面的方法，可以改为4K自动增加的，但是需要花时间，而且暂时这个下载器的速度，也不是很慢，所以暂时先不考虑，日后为了提升速度，可以增加这个！
							
							#define SWD_AUTO_ADDRESS_ADD_VALUE 0x400   // 特别警告：STM32F0系列的地址自动增加大小为：1024个，而不是F1和F4的4096个！
							if( ((Address + tmp12) %SWD_AUTO_ADDRESS_ADD_VALUE) == 0 ) // 说明到了4k临界点，需要重新写地址
							{
								writeAP( AP_TAR, Address + tmp12 );  // 地址！
							}
							
							if( (Address+tmp12)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 24) + ( W25Q4KBuf.databuf4K[tmp12] << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
							}
						}
						{ // 程序进度区！程序进度是4K大小！进度梯度是10%为进度！
							// 这个使用emWin应该是一个简单的问题！
							float tmp13 = (float)((float)((float)tmp11+1.0)/(float)HowMany4K);
							for( float tmp14 = 0.0;tmp14 <= (float)1.0; tmp14 += 0.1 )
							{
								if( tmp13 <= tmp14 ) // 找到第一次开始小于等于的值！
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
				{ // 写滚码区
					u32 programTimesTmp15 = 0;  // 实际已经编程次数
					u32 valueTmp16 = 0; 				// 实际写入的滚码值
					if( userfile.RollingCodeFunction == 0x55 ) // 表示需要操作滚码区
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
									programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + Tmp16;  // 实际编程次数
									break;
								}
							}
						}
						valueTmp16 = userfile.RollingCodeStartValue + ( userfile.RollingCodeStepValue* programTimesTmp15 );
						writeAP( AP_TAR, userfile.RollingCodeStartAddress );  // 地址！
						{
							if( (userfile.RollingCodeStartAddress)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
							}
						}
						{
							writeAP( AP_TAR, userfile.RollingCodeStartAddress+2 );  // 地址！
							{
								if( (userfile.RollingCodeStartAddress+2)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
								}
								else
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
								}
							}
						}
					
						{ // 检验滚码区
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // 访问寄存器
							writeAP( AP_CSW,0x2 | 0x10);   // 长度为32bit，同时地址自动增加
							writeAP( AP_TAR,userfile.RollingCodeStartAddress);   // 写目标地址
							readAP( AP_DRW,&apId);  // 第一次数据，不要！
							readDP( DP_RDBUFF, &apId );
							if( apId != valueTmp16 )
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    滚码校验失败     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "Roll code check failed");
								}
								returnNumber = 0x00;  // 检测错误
								return returnNumber;
							}
						}
					}
				}
				{ // 校验程序区，暂时只使用和校验！
					switch( userfile.FlashCheckWay )
					{
						case 0x00: // 不检验
							if( L_ENGLISH_VER != 0x55AA )
							{
								LCD_ShowString(0,12*11,12,RED, "     不校验FLASH     ");
							}
							else
							{
								LCD_ShowString(0,12*11,12,RED, "   not check FLASH   ");
							}
							returnNumber = 0x55AA;
							break;
						case 0x01: // 逐个检验
							break;
						case 0x02: // CRC校验（和校验） -- 日后应该只会用这种校验方式，因为其它的也没有必要性了！
							{ // 这样的校验方式有这样的要求：起始地址4字节对齐，程序大小4字节倍数！这个日后务必要保证，如果保证不了，可以从软件上解决这个问题！
								u16 sumTmp15 = 0;  // 最后的和校验程序
								writeDP( DP_SELECT, 0x00 );  // 访问寄存器
								writeAP( AP_CSW,0x2 | 0x10);   // 长度为32bit，同时地址自动增加
								u32 addressTmp16 = userfile.ProgramStartAddress;
								writeAP( AP_TAR,addressTmp16);   // 写目标地址
								
								u32 apId = 0;
								readAP( AP_DRW,&apId);  // 第一次数据，不要！
								
								for( ;addressTmp16 < ( userfile.ProgramStartAddress + userfile.ProgramSize );addressTmp16 += 4 )
								{
//									u32 apId = 0;
//									if( ( addressTmp16 %0x1000 ) == 0 )
//									{
//										writeAP( AP_TAR,addressTmp16);   // 写目标地址
//									}
//									readAP( AP_DRW,&apId);  // 关于读数据的话，每次只能读上一次的。就是有一次延时。
//									readDP( DP_RDBUFF, &apId );
//									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
									if( ( addressTmp16 %SWD_AUTO_ADDRESS_ADD_VALUE ) == 0 )
									{
										writeAP( AP_TAR,addressTmp16);   // 写目标地址
										readAP( AP_DRW,&apId); 
									}
									readAP( AP_DRW,&apId);  // 这次数据是上次的。但是也是需要的。
									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
								}
								if( sumTmp15 == userfile.SumValueOfProgram )
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12, BLACK,"   FLASH 校验成功   ");
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
										LCD_ShowString(0,12*11,12,BLACK, "   FLASH 校验失败    ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check failure ");
									}
									returnNumber = 0x1122;
								}
							}
							break;
						case 0x03: // MD5值检验
							break;
						default:
							break;
					}
				}
				{ // 写滚码，且校验滚码区！！
					// 为了使滚码操作比较方便：要求1：必须在4字节对齐地址上，必须使用小端模式！
					{ // 清除滚码地址
						
					}
					{ // 写滚码
						
					}
					{ // 校验滚码，使用每个对比！
						
					}
				}
				{ // 操作保护区！(其实这里应该加一个两个都不使能的操作！)
					switch( userfile.OptionBytesFunction )
					{
						case 0x55: // 使能选项字节，日后功能开放！
							break;
						case 0xAA: // 使能读保护
							stm32f0Protection( ENABLE,DISABLE );
							break;
						case 0x11: // 使能写保护
							stm32f0Protection( DISABLE,ENABLE );
							break;
						case 0x1A: // 使能读写保护
							stm32f0Protection( ENABLE,ENABLE );
							break;
					}
				}
				{
					writeAP( AP_CSW,0x23000002); // 写
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, _0b10000000);  // 禁止编程和锁定
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
				{ // 检查保护区！
							OS_ERR err;
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // 访问寄存器
							writeAP( AP_CSW,0x2 );   // 长度为32bit，同时地址自动增加
							writeAP( AP_TAR,(u32)&OB->RDP);   // 写目标地址
							readAP( AP_DRW,&apId);  // 第一次数据，不要！
							readDP( DP_RDBUFF, &apId );
							if( apId == 0xFFFF5AA5 ) // 如果能操作到这一步表明没有设置读保护
							{
								{ // 检查写保护
									writeAP( AP_TAR,(u32)&OB->WRP0);
									readAP( AP_DRW,&apId);
									readDP( DP_RDBUFF, &apId );
									
									if( apId != 0xFFFFFFFF)  // 说明存在写保护
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      发现写保护      ");
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
									if( apId != 0xFFFFFFFF ) // 说明存在写保护
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      发现读保护      ");
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
							{ // 设置了读保护，要清除！
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*10,12,RED, "      发现读保护      ");
								}
								else
								{
									LCD_ShowString(0,12*10,12,RED, "      read protect    ");
								}
								dealTmp0:
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    清除中... 2S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 2S    ");
								}
								{ // 清除SWD的错误！
									writeDP( DP_ABORT,0x1F);
									
									stm32f1Protection( DISABLE,DISABLE );
								}
								OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    清除中... 1S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 1S    ");
								}
								OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    清除中... 0S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 0S    ");
								}
								
								LCD_ShowString(0,12*10,12,RED, "  Protection Cleared ");
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    请断电重新下载   ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "Download again without power");
								}
								return 0xFFBB;
							}
					}
			}
			
			{ // 清除Flash
				if( userfile.ChipEraseWay == 00 )  // 全擦除
				{
					clearAllFlash(  );  // 这个是清除全部Flash的操作
				}
				else	// 块擦除，分为小容量大容量和大容量，但是大容量也可以用1KB的来操作，这样可以简化，但是浪费时间，第一个版本不用太在意！
							// 除此之外，还有滚码所在的地址！
				{
					u32 startAddressTmp0 = 0x08000000; // 起始页所在地址
					u32 endAddressTmp1 = 0x08000000;   // 结束页所在地址
					u32 startRollingAddressTmp3 = 0x08000000;
					u32 endRollingAddressTmp4 = 0x08000000;
					for( u32 tmp5 = 0;tmp5<1024;tmp5++ ) // 1024：最大为1M大小！
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
						if( (userfile.RollingCodeStartAddress + 32 ) < ( 0x08000000 + 0x400 * tmp8 ) )  // 这个的SIZE只有32
						{
							endRollingAddressTmp4 = 0x08000000 + 0x400 * (tmp8 - 1);
							break;
						}
					}
					{ // 用SWD清除Flash操作
						writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
	
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY1);
						
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY2);
						
						writeAP( AP_CSW,0x00000002);   // 读
						u32 readWord0 = 0;
						do
						{
							writeAP(AP_TAR, (u32)&FLASH->SR.All);
							readAP(AP_DRW,&readWord0);
							readDP(DP_RDBUFF,&readWord0);
						}while(readWord0&0x1); // 当为1的时候说明没有操作完成！
						
						for( u32 addressTmp9 = startAddressTmp0; addressTmp9<= endAddressTmp1; addressTmp9 += 0x400 )  // 清除程序区
						{
							writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000010 );  // 页擦除使能
							
							writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
							writeAP( AP_DRW, addressTmp9 );  // 写入地址
							
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b01000010 );  // 开始
							
							writeAP( AP_CSW,0x00000002);   // 读
							u32 readWord1 = 0;
							do
							{
								writeAP(AP_TAR, (u32)&FLASH->SR.All);
								readAP(AP_DRW,&readWord1);
								readDP(DP_RDBUFF,&readWord1);
							}while(readWord1&0x1); // 当为1的时候说明没有操作完成！
						}
						{ // 恢复现场
							writeAP( AP_CSW,0x23000002);  
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000000 );
						}
						if( userfile.RollingCodeFunction == 0x55 ) // 使能滚码操作，只有当滚码操作使能的时候，再操作，防止访问了错误的地址！
						{
							for( u32 addressTmp10 = startRollingAddressTmp3; addressTmp10<= endRollingAddressTmp4; addressTmp10 += 0x400 )  // 清除滚码区
							{
								writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000010 );  // 页擦除使能
								
								writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
								writeAP( AP_DRW, addressTmp10 );  // 写入地址
								
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b01000010 );  // 开始
								
								writeAP( AP_CSW,0x00000002);   // 读
								u32 readWord1 = 0;
								do
								{
									writeAP(AP_TAR, (u32)&FLASH->SR.All);
									readAP(AP_DRW,&readWord1);
									readDP(DP_RDBUFF,&readWord1);
								}while(readWord1&0x1); // 当为1的时候说明没有操作完成！
							}
							{ // 恢复现场
								writeAP( AP_CSW,0x23000002);  
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000000 );
							}
						}
						{ // 锁定Flash
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b10000000 );
						}
					}
				}
			}
			{ // 写Flash，同时要写滚码操作区！
				
				{  // 解锁Flash操作
					writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY1);
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY2);
				}
				{ // 使能编辑位
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, 0x1);  // 使能编程操作！
				}
				
				{ // 写程序区
					u32 HowMany4K = 0;
					if( userfile.ProgramSize % 0x1000 != 0 )
						HowMany4K = userfile.ProgramSize/0x1000 + 1;  // 之前的全都写成：4000，实际应该是：
					else
						HowMany4K = userfile.ProgramSize/0x1000;
					
					for( u32 tmp11 = 0;tmp11<HowMany4K; tmp11++ )  // 写Flash的操作，每次操作4K字节
					{
						u32 Address = userfile.ProgramStartAddress + tmp11*0x1000;  // 起始操作地址
						W25QXX_Read( W25Q4KBuf.databuf4K , userfile.ProgramSaveAtW25QAddress + tmp11*0x1000, 0x1000); // 4k大小在16进行下，就是0x1000
						
						writeAP( AP_CSW,0x23000001 | 0x10 ); // 长度为16bit,写，且地址自动增加！
						writeAP( AP_TAR, Address );  // 地址！
						for( u32 tmp12 = 0;tmp12 <0x1000; tmp12 += 2 ) // 写入4K
						{
							{ // 实测，可以把下面的给省略掉，这样可以提升速度一个测试是：从56S - 36S！因为写单个可能根本不需要这样！但是这个暂时保留，日后防止速度上来后，影响速度！
//								writeAP( AP_CSW,0x00000002 );   // 读
//								do
//								{
//									writeAP(AP_TAR, (u32)&FLASH->SR.All);
//									readAP(AP_DRW,&readWord0);
//									readDP(DP_RDBUFF,&readWord0);
//								}while(readWord0&0x1); // 当为1的时候说明没有操作完成！
							}
							// 下面的方法，可以改为4K自动增加的，但是需要花时间，而且暂时这个下载器的速度，也不是很慢，所以暂时先不考虑，日后为了提升速度，可以增加这个！
							if( ((Address + tmp12) %0x1000) == 0 ) // 说明到了4k临界点，需要重新写地址
							{
								writeAP( AP_TAR, Address + tmp12 );  // 地址！
							}
							
							
							
							
							
							if( (Address+tmp12)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 24) + ( W25Q4KBuf.databuf4K[tmp12] << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
							}
						}
						{ // 程序进度区！程序进度是4K大小！进度梯度是10%为进度！
							// 这个使用emWin应该是一个简单的问题！
							float tmp13 = (float)((float)((float)tmp11+1.0)/(float)HowMany4K);
							for( float tmp14 = 0.0;tmp14 <= (float)1.0; tmp14 += 0.1 )
							{
								if( tmp13 <= tmp14 ) // 找到第一次开始小于等于的值！
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
				{ // 写滚码区
					u32 programTimesTmp15 = 0;  // 实际已经编程次数
					u32 valueTmp16 = 0; 				// 实际写入的滚码值
					if( userfile.RollingCodeFunction == 0x55 ) // 表示需要操作滚码区
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
									programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + Tmp16;  // 实际编程次数
									break;
								}
							}
						}
						valueTmp16 = userfile.RollingCodeStartValue + ( userfile.RollingCodeStepValue* programTimesTmp15 );
						writeAP( AP_TAR, userfile.RollingCodeStartAddress );  // 地址！
						{
							if( (userfile.RollingCodeStartAddress)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
							}
						}
						{
							writeAP( AP_TAR, userfile.RollingCodeStartAddress+2 );  // 地址！
							{
								if( (userfile.RollingCodeStartAddress+2)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
								}
								else
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
								}
							}
						}
					
						{ // 检验滚码区
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // 访问寄存器
							writeAP( AP_CSW,0x2 | 0x10);   // 长度为32bit，同时地址自动增加
							writeAP( AP_TAR,userfile.RollingCodeStartAddress);   // 写目标地址
							readAP( AP_DRW,&apId);  // 第一次数据，不要！
							readDP( DP_RDBUFF, &apId );
							if( apId != valueTmp16 )
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    滚码校验失败     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "Roll code check failed");
								}
								returnNumber = 0x00;  // 检测错误
								return returnNumber;
							}
						}
					}
				}
				{ // 校验程序区，暂时只使用和校验！
					switch( userfile.FlashCheckWay )
					{
						case 0x00: // 不检验
							if( L_ENGLISH_VER != 0x55AA )
							{
								LCD_ShowString(0,12*11,12,RED, "     不校验FLASH     ");
							}
							else
							{
								LCD_ShowString(0,12*11,12,RED, "   not check FLASH   ");
							}
							returnNumber = 0x55AA;
							break;
						case 0x01: // 逐个检验
							break;
						case 0x02: // CRC校验（和校验） -- 日后应该只会用这种校验方式，因为其它的也没有必要性了！
							{ // 这样的校验方式有这样的要求：起始地址4字节对齐，程序大小4字节倍数！这个日后务必要保证，如果保证不了，可以从软件上解决这个问题！
								u16 sumTmp15 = 0;  // 最后的和校验程序
								writeDP( DP_SELECT, 0x00 );  // 访问寄存器
								writeAP( AP_CSW,0x2 | 0x10);   // 长度为32bit，同时地址自动增加
								u32 addressTmp16 = userfile.ProgramStartAddress;
								writeAP( AP_TAR,addressTmp16);   // 写目标地址
								
								u32 apId = 0;
								readAP( AP_DRW,&apId);  // 第一次数据，不要！
								
								for( ;addressTmp16 < ( userfile.ProgramStartAddress + userfile.ProgramSize );addressTmp16 += 4 )
								{
//									u32 apId = 0;
//									if( ( addressTmp16 %0x1000 ) == 0 )
//									{
//										writeAP( AP_TAR,addressTmp16);   // 写目标地址
//									}
//									readAP( AP_DRW,&apId);  // 关于读数据的话，每次只能读上一次的。就是有一次延时。
//									readDP( DP_RDBUFF, &apId );
//									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
									if( ( addressTmp16 %0x1000 ) == 0 )
									{
										writeAP( AP_TAR,addressTmp16);   // 写目标地址
										readAP( AP_DRW,&apId); 
									}
									readAP( AP_DRW,&apId);  // 这次数据是上次的。但是也是需要的。
									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
								}
								if( sumTmp15 == userfile.SumValueOfProgram )
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12, BLACK,"   FLASH 校验成功   ");
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
										LCD_ShowString(0,12*11,12,BLACK, "   FLASH 校验失败    ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check failure ");
									}
									returnNumber = 0x1122;
									return returnNumber;  // 如果此时出错，则进行返回，因为后面的语句可能使能保护，尤其断电的时候，需要断电重新下载（比如GD32，）此时会有问题！
								}
							}
							break;
						case 0x03: // MD5值检验
							break;
						default:
							break;
					}
				}
				{ // 写滚码，且校验滚码区！！
					// 为了使滚码操作比较方便：要求1：必须在4字节对齐地址上，必须使用小端模式！
					{ // 清除滚码地址
						
					}
					{ // 写滚码
						
					}
					{ // 校验滚码，使用每个对比！
						
					}
				}
				{ // 操作保护区！(其实这里应该加一个两个都不使能的操作！)
					switch( userfile.OptionBytesFunction )
					{
						case 0x55: // 使能选项字节，日后功能开放！
							break;
						case 0xAA: // 使能读保护
							stm32f1Protection( ENABLE,DISABLE );
							break;
						case 0x11: // 使能写保护
							stm32f1Protection( DISABLE,ENABLE );
							break;
						case 0x1A: // 使能读写保护
							stm32f1Protection( ENABLE,ENABLE );
							break;
					}
				}
				{
					writeAP( AP_CSW,0x23000002); // 写
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, _0b10000000);  // 禁止编程和锁定
				}
				{
					resetAndHaltTarget( );
					runTarget( );
				}
			}
			
			return returnNumber;
}

// F3系列的支持：2018.07.09 - 暂时没有通过测试

u32 DownloadFlashOfSTM32F3xx( _FileInformation userfile )
{
	u32 returnNumber = 0;
	{ // 
				{ // 检查保护区！
							OS_ERR err;
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // 访问寄存器
							writeAP( AP_CSW,0x2 );   // 长度为32bit，同时地址自动增加
							writeAP( AP_TAR,(u32)&OB->RDP);   // 写目标地址
							readAP( AP_DRW,&apId);  // 第一次数据，不要！
							readDP( DP_RDBUFF, &apId );
							if( (apId & 0xFFFF) == 0x55AA ) // 如果能操作到这一步表明没有设置读保护
							{
								{ // 检查写保护
									writeAP( AP_TAR,(u32)&OB->WRP0);
									readAP( AP_DRW,&apId);
									readDP( DP_RDBUFF, &apId );
									
									if( apId != 0xFFFFFFFF)  // 说明存在写保护
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      发现写保护      ");
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
									if( apId != 0xFFFFFFFF ) // 说明存在写保护
									{
										if( L_ENGLISH_VER != 0x55AA )
										{
											LCD_ShowString(0,12*10,12,RED, "      发现读保护      ");
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
							{ // 设置了读保护，要清除！
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*10,12,RED, "      发现读保护      ");
								}
								else
								{
									LCD_ShowString(0,12*10,12,RED, "      read protect    ");
								}
								dealTmp0:
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    清除中... 2S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 2S    ");
								}
								{ // 清除SWD的错误！
									writeDP( DP_ABORT,0x1F);
									stm32f3Protection( DISABLE,DISABLE );
								}
								OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    清除中... 2S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 2S    ");
								}
								OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    清除中... 2S     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED,   "    Cleaing... 2S    ");
								}
								
								LCD_ShowString(0,12*10,12,RED, "  Protection Cleared ");
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    请断电重新下载   ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "Download again without power");
								}
								return 0x00;
							}
					}
			}
			
			{ // 清除Flash
				if( userfile.ChipEraseWay == 00 )  // 全擦除
				{
					clearAllFlash(  );  // 这个是清除全部Flash的操作
				}
				else	// 块擦除，分为小容量大容量和大容量，但是大容量也可以用1KB的来操作，这样可以简化，但是浪费时间，第一个版本不用太在意！
							// 除此之外，还有滚码所在的地址！
				{
					u32 startAddressTmp0 = 0x08000000; // 起始页所在地址
					u32 endAddressTmp1 = 0x08000000;   // 结束页所在地址
					u32 startRollingAddressTmp3 = 0x08000000;
					u32 endRollingAddressTmp4 = 0x08000000;
					for( u32 tmp5 = 0;tmp5<1024;tmp5++ ) // 1024：最大为1M大小！
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
						if( (userfile.RollingCodeStartAddress + 32 ) < ( 0x08000000 + 0x400 * tmp8 ) )  // 这个的SIZE只有32
						{
							endRollingAddressTmp4 = 0x08000000 + 0x400 * (tmp8 - 1);
							break;
						}
					}
					{ // 用SWD清除Flash操作
						writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
	
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY1);
						
						writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
						writeAP(AP_DRW, FLASH_KEY2);
						
						writeAP( AP_CSW,0x00000002);   // 读
						u32 readWord0 = 0;
						do
						{
							writeAP(AP_TAR, (u32)&FLASH->SR.All);
							readAP(AP_DRW,&readWord0);
							readDP(DP_RDBUFF,&readWord0);
						}while(readWord0&0x1); // 当为1的时候说明没有操作完成！
						
						for( u32 addressTmp9 = startAddressTmp0; addressTmp9<= endAddressTmp1; addressTmp9 += 0x400 )  // 清除程序区
						{
							writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000010 );  // 页擦除使能
							
							writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
							writeAP( AP_DRW, addressTmp9 );  // 写入地址
							
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b01000010 );  // 开始
							
							writeAP( AP_CSW,0x00000002);   // 读
							u32 readWord1 = 0;
							do
							{
								writeAP(AP_TAR, (u32)&FLASH->SR.All);
								readAP(AP_DRW,&readWord1);
								readDP(DP_RDBUFF,&readWord1);
							}while(readWord1&0x1); // 当为1的时候说明没有操作完成！
						}
						{ // 恢复现场
							writeAP( AP_CSW,0x23000002);  
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b00000000 );
						}
						if( userfile.RollingCodeFunction == 0x55 ) // 使能滚码操作，只有当滚码操作使能的时候，再操作，防止访问了错误的地址！
						{
							for( u32 addressTmp10 = startRollingAddressTmp3; addressTmp10<= endRollingAddressTmp4; addressTmp10 += 0x400 )  // 清除滚码区
							{
								writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000010 );  // 页擦除使能
								
								writeAP( AP_TAR, (u32)&FLASH->AR.Address_W );
								writeAP( AP_DRW, addressTmp10 );  // 写入地址
								
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b01000010 );  // 开始
								
								writeAP( AP_CSW,0x00000002);   // 读
								u32 readWord1 = 0;
								do
								{
									writeAP(AP_TAR, (u32)&FLASH->SR.All);
									readAP(AP_DRW,&readWord1);
									readDP(DP_RDBUFF,&readWord1);
								}while(readWord1&0x1); // 当为1的时候说明没有操作完成！
							}
							{ // 恢复现场
								writeAP( AP_CSW,0x23000002);  
								writeAP( AP_TAR, (u32)&FLASH->CR.All );
								writeAP( AP_DRW, _0b00000000 );
							}
						}
						{ // 锁定Flash
							writeAP( AP_TAR, (u32)&FLASH->CR.All );
							writeAP( AP_DRW, _0b10000000 );
						}
					}
				}
			}
			{ // 写Flash，同时要写滚码操作区！
				
				{  // 解锁Flash操作
					writeAP( AP_CSW,0x23000002);   // 一定要先提前写上这个，不仅仅表示是32位，同时表明访问的方向是写访问！否则会报错！！
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY1);
					
					writeAP(AP_TAR, (u32)&FLASH->KEYR.FlashProgramAndEraseControllerKey_W);
					writeAP(AP_DRW, FLASH_KEY2);
				}
				{ // 使能编辑位
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, 0x1);  // 使能编程操作！
				}
				
				{ // 写程序区
					u32 HowMany4K = 0;
					if( userfile.ProgramSize % 0x1000 != 0 )
						HowMany4K = userfile.ProgramSize/0x1000 + 1;
					else
						HowMany4K = userfile.ProgramSize/0x1000;
					
					for( u32 tmp11 = 0;tmp11<HowMany4K; tmp11++ )  // 写Flash的操作，每次操作4K字节
					{
						u32 Address = userfile.ProgramStartAddress + tmp11*0x1000;  // 起始操作地址
						W25QXX_Read( W25Q4KBuf.databuf4K , userfile.ProgramSaveAtW25QAddress + tmp11*0x1000, 0x1000); // 4k大小在16进行下，就是0x1000
						
						writeAP( AP_CSW,0x23000001 | 0x10 ); // 长度为16bit,写，且地址自动增加！
						writeAP( AP_TAR, Address );  // 地址！
						for( u32 tmp12 = 0;tmp12 <0x1000; tmp12 += 2 ) // 写入4K
						{
							{ // 实测，可以把下面的给省略掉，这样可以提升速度一个测试是：从56S - 36S！因为写单个可能根本不需要这样！但是这个暂时保留，日后防止速度上来后，影响速度！
//								writeAP( AP_CSW,0x00000002 );   // 读
//								do
//								{
//									writeAP(AP_TAR, (u32)&FLASH->SR.All);
//									readAP(AP_DRW,&readWord0);
//									readDP(DP_RDBUFF,&readWord0);
//								}while(readWord0&0x1); // 当为1的时候说明没有操作完成！
							}
							// 下面的方法，可以改为4K自动增加的，但是需要花时间，而且暂时这个下载器的速度，也不是很慢，所以暂时先不考虑，日后为了提升速度，可以增加这个！
							if( ((Address + tmp12) %0x1000) == 0 ) // 说明到了4k临界点，需要重新写地址
							{
								writeAP( AP_TAR, Address + tmp12 );  // 地址！
							}
							
							if( (Address+tmp12)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 24) + ( W25Q4KBuf.databuf4K[tmp12] << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
							}
						}
						{ // 程序进度区！程序进度是4K大小！进度梯度是10%为进度！
							// 这个使用emWin应该是一个简单的问题！
							float tmp13 = (float)((float)((float)tmp11+1.0)/(float)HowMany4K);
							for( float tmp14 = 0.0;tmp14 <= (float)1.0; tmp14 += 0.1 )
							{
								if( tmp13 <= tmp14 ) // 找到第一次开始小于等于的值！
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
				{ // 写滚码区
					u32 programTimesTmp15 = 0;  // 实际已经编程次数
					u32 valueTmp16 = 0; 				// 实际写入的滚码值
					if( userfile.RollingCodeFunction == 0x55 ) // 表示需要操作滚码区
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
									programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + Tmp16;  // 实际编程次数
									break;
								}
							}
						}
						valueTmp16 = userfile.RollingCodeStartValue + ( userfile.RollingCodeStepValue* programTimesTmp15 );
						writeAP( AP_TAR, userfile.RollingCodeStartAddress );  // 地址！
						{
							if( (userfile.RollingCodeStartAddress)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
							}
							else
							{
								writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
							}
						}
						{
							writeAP( AP_TAR, userfile.RollingCodeStartAddress+2 );  // 地址！
							{
								if( (userfile.RollingCodeStartAddress+2)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
								}
								else
								{
									writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
								}
							}
						}
					
						{ // 检验滚码区
							u32 apId = 0;
							writeDP( DP_SELECT, 0x00 );  // 访问寄存器
							writeAP( AP_CSW,0x2 | 0x10);   // 长度为32bit，同时地址自动增加
							writeAP( AP_TAR,userfile.RollingCodeStartAddress);   // 写目标地址
							readAP( AP_DRW,&apId);  // 第一次数据，不要！
							readDP( DP_RDBUFF, &apId );
							if( apId != valueTmp16 )
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*11,12,RED, "    滚码校验失败     ");
								}
								else
								{
									LCD_ShowString(0,12*11,12,RED, "Roll code check failed");
								}
								returnNumber = 0x00;  // 检测错误
								return returnNumber;
							}
						}
					}
				}
				{ // 校验程序区，暂时只使用和校验！
					switch( userfile.FlashCheckWay )
					{
						case 0x00: // 不检验
							if( L_ENGLISH_VER != 0x55AA )
							{
								LCD_ShowString(0,12*11,12,RED, "     不校验FLASH     ");
							}
							else
							{
								LCD_ShowString(0,12*11,12,RED, "   not check FLASH   ");
							}
							returnNumber = 0x55AA;
							break;
						case 0x01: // 逐个检验
							break;
						case 0x02: // CRC校验（和校验） -- 日后应该只会用这种校验方式，因为其它的也没有必要性了！
							{ // 这样的校验方式有这样的要求：起始地址4字节对齐，程序大小4字节倍数！这个日后务必要保证，如果保证不了，可以从软件上解决这个问题！
								u16 sumTmp15 = 0;  // 最后的和校验程序
								writeDP( DP_SELECT, 0x00 );  // 访问寄存器
								writeAP( AP_CSW,0x2 | 0x10);   // 长度为32bit，同时地址自动增加
								u32 addressTmp16 = userfile.ProgramStartAddress;
								writeAP( AP_TAR,addressTmp16);   // 写目标地址
								
								u32 apId = 0;
								readAP( AP_DRW,&apId);  // 第一次数据，不要！
								
								for( ;addressTmp16 < ( userfile.ProgramStartAddress + userfile.ProgramSize );addressTmp16 += 4 )
								{
//									u32 apId = 0;
//									if( ( addressTmp16 %0x1000 ) == 0 )
//									{
//										writeAP( AP_TAR,addressTmp16);   // 写目标地址
//									}
//									readAP( AP_DRW,&apId);  // 关于读数据的话，每次只能读上一次的。就是有一次延时。
//									readDP( DP_RDBUFF, &apId );
//									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
									if( ( addressTmp16 %0x1000 ) == 0 )
									{
										writeAP( AP_TAR,addressTmp16);   // 写目标地址
										readAP( AP_DRW,&apId); 
									}
									readAP( AP_DRW,&apId);  // 这次数据是上次的。但是也是需要的。
									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
								}
								if( sumTmp15 == userfile.SumValueOfProgram )
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12, BLACK,"   FLASH 校验成功   ");
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
										LCD_ShowString(0,12*11,12,BLACK, "   FLASH 校验失败    ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check failure ");
									}
									returnNumber = 0x1122;
								}
							}
							break;
						case 0x03: // MD5值检验
							break;
						default:
							break;
					}
				}
				{ // 写滚码，且校验滚码区！！
					// 为了使滚码操作比较方便：要求1：必须在4字节对齐地址上，必须使用小端模式！
					{ // 清除滚码地址
						
					}
					{ // 写滚码
						
					}
					{ // 校验滚码，使用每个对比！
						
					}
				}
				{ // 操作保护区！(其实这里应该加一个两个都不使能的操作！)
					switch( userfile.OptionBytesFunction )
					{
						case 0x55: // 使能选项字节，日后功能开放！
							break;
						case 0xAA: // 使能读保护
							stm32f1Protection( ENABLE,DISABLE );
							break;
						case 0x11: // 使能写保护
							stm32f1Protection( DISABLE,ENABLE );
							break;
						case 0x1A: // 使能读写保护
							stm32f1Protection( ENABLE,ENABLE );
							break;
					}
				}
				{
					writeAP( AP_CSW,0x23000002); // 写
					writeAP(AP_TAR, (u32)&FLASH->CR.All);
					writeAP(AP_DRW, _0b10000000);  // 禁止编程和锁定
				}
				{
					resetAndHaltTarget( );
					runTarget( );
				}
			}
			
			return returnNumber;
}

/**
 *  @B F2系列的下载函数，根据对比手册，发现F2和F4非常的相似，所以暂时使用F4的函数来改改！
 *  
 */

u32 DownloadFlashOfSTM32F2xx( _FileInformation userfile )
{
	u32 returnNumber = 0;
//	u32 F4FlashMemorySize = 0;  // 在操作擦除操作和保护操作时，必须要知道其大小，否则会出现没有定义的值！
	{ // 
//		{ // 检查Flash的大小。
//			u32 apId = 0;
//			writeDP( DP_SELECT, 0x00 );  // 访问寄存器
//			writeAP( AP_CSW,0x2 );   // 长度为32bit，同时地址自动增加
//			writeAP( AP_TAR,(u32)(0x1FFF7A22));   // 写目标地址
//			readAP( AP_DRW,&apId);  // 第一次数据，不要！
//			readDP( DP_RDBUFF, &apId );
//			F4FlashMemorySize = apId>>16;
//		}
		{ // 检查保护区！
					OS_ERR err;
					u32 apId = 0;
					writeDP( DP_SELECT, 0x00 );  // 访问寄存器
					writeAP( AP_CSW,0x2 );   // 长度为32bit，同时地址自动增加
					writeAP( AP_TAR,(u32)&F4_FLASH->OPTCR);   // 写目标地址
					readAP( AP_DRW,&apId);  // 第一次数据，不要！
					readDP( DP_RDBUFF, &apId );
			
			
					if( ((apId>>8)&(0xFF)) == 0xAA ) // 没有激活读保护
					{
						{ // 检查写保护
							if( ((apId>>16)&(0x0FFF)) != 0x0FFF )  // 说明存在写保护
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*10,12,RED, "      发现写保护      ");
								}
								else
								{
									LCD_ShowString(0,12*10,12,RED, "  have write protect  ");
								}
								goto dealTmp0;
							}
							
//							if( F4FlashMemorySize > 0x400 ) // 大于1M的存储
//							{
//								writeAP( AP_TAR,(u32)&F4_FLASH->OPTCR1);
//								readAP( AP_DRW,&apId);
//								readDP( DP_RDBUFF, &apId );
//								if( ((apId>>16)&(0x0FFF)) != 0x0FFF )  // 说明存在写保护
//								{
//									LCD_ShowString(0,12*10,12,RED, "   WRITE protection  ");
//									goto dealTmp0;
//								}
//							}
						}
					}
					else if( ((apId>>8)&(0xFF)) == 0xCC )  // 不可以恢复的读保护
					{
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*10,12,RED, "         警告        ");
							LCD_ShowString(0,12*11,12,RED, "       读保护-LV2    ");
						}
						else
						{
							LCD_ShowString(0,12*10,12,RED, "       warning     ");
							LCD_ShowString(0,12*11,12,RED, "    ReadProtect-LV2  ");
						}
						return 0x00;
					}
					else  // 设置了可以清除的读保护
					{ // 设置了读保护，要清除！
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*10,12,RED, "      发现读保护      ");
						}
						else
						{
							LCD_ShowString(0,12*10,12,RED, "      read protect    ");
						}
						dealTmp0:
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*11,12,RED, "    清除中...      ");
						}
						else
						{
							LCD_ShowString(0,12*11,12,RED,   "    Cleaing...     ");
						}
						{ // 清除SWD的错误！
							writeDP( DP_ABORT,0x1F);
							stm32f2Protection( DISABLE,DISABLE );
						}
						OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
						OSTimeDlyHMSM(0,0,0,1000,OS_OPT_TIME_HMSM_NON_STRICT,&err);
						
						LCD_ShowString(0,12*10,12,RED, "  Protection Cleared ");
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*11,12,RED, "    请断电重新下载   ");
						}
						else
						{
							LCD_ShowString(0,12*11,12,RED, "Download again without power");
						}
												
						return 0xFFBB;
					}
			}
	}
	
	{ // 清除Flash
		if( userfile.ChipEraseWay == 00 )  // 全擦除
		{
			writeAP( AP_CSW,0x23000002);   // 32位写访问
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0x45670123 );
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0xCDEF89AB );
			
			writeAP( AP_CSW,0x00000002);   // 读
			u32 readWord0 = 0;
			do
			{
				writeAP(AP_TAR, (u32)&F4_FLASH->SR);
				readAP(AP_DRW,&readWord0);
				readDP(DP_RDBUFF,&readWord0);
			}while((readWord0>>16)&0x1); // 当为1的时候说明没有操作完成！
			
//			if( F4FlashMemorySize > 0x400 )  // 全擦除操作需要注意容量大小   ---  这里是不是写错了？
//			{
//				writeAP( AP_CSW,0x23000002);   // 写32位
//				writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // 擦除操作，同时设置PSIZE值为2，表示为32位操作大小！
//				writeAP( AP_DRW, 0x0204 );
//				
//				writeAP( AP_TAR, (u32)&F4_FLASH->CR );
//				writeAP( AP_DRW, 0x0204 + ( 1<<16 )); // 开始操作
//				
//				
//				writeAP( AP_CSW,0x00000002);   // 读
//				u32 readWord1 = 0;
//				do
//				{
//					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
//					readAP(AP_DRW,&readWord1);
//					readDP(DP_RDBUFF,&readWord1);
//				}while((readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
//			}
//			else
//			{	
				writeAP( AP_CSW,0x23000002);   // 写32位
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // 擦除操作，同时设置PSIZE值为2，表示为32位操作大小！
				writeAP( AP_DRW, 0x0204 );   // 这里的操作不一样。
				
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );
				writeAP( AP_DRW, 0x0204 + ( 1<<16 )); // 开始操作
				
				
				writeAP( AP_CSW,0x00000002);   // 读
				u32 readWord1 = 0;
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord1);
					readDP(DP_RDBUFF,&readWord1);
				}while((readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
//			}
		}
		else	// 块擦除，分为小容量大容量和大容量，但是大容量也可以用1KB的来操作，这样可以简化，但是浪费时间，第一个版本不用太在意！
					// 除此之外，还有滚码所在的地址！
					// 注意F4的块问题。因为分配均匀，所以会比较不一样。可以考虑用表结构！
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
			
			u32 startAddressTmp0 = 0x08000000; // 起始页所在地址
			u32 endAddressTmp1 = 0x08000000;   // 结束页所在地址
			u32 startAddressPage = 0;
			u32 endAddressPage = 0;
																
			u32 startRollingAddressTmp3 = 0x08000000;
			u32 endRollingAddressTmp4 = 0x08000000;
			u32 startRollingAddressPage = 0;
			u32 endRollingdressPage = 0;
																
			for( u32 tmp5 = 0;tmp5<1024;tmp5++ ) // 2048：最大为2M大小！
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
				if( (userfile.RollingCodeStartAddress + 32 ) < ( 0x08000000 + 0x400 * tmp8 ) )  // 这个的SIZE只有32
				{
					endRollingAddressTmp4 = 0x08000000 + 0x400 * (tmp8 - 1);
					break;
				}
			}
			{ // 找扇区所在的页号
				for( u8 Temp0 = 0;Temp0<4;Temp0++ ) // 总共查找4次
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
			{ // 用SWD清除Flash操作
				writeAP( AP_CSW,0x23000002);   // 32位写访问
			
				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
				writeAP( AP_DRW, 0x45670123 );
				
				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
				writeAP( AP_DRW, 0xCDEF89AB );
				
				writeAP( AP_CSW,0x00000002);   // 读
				u32 readWord0 = 0;
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord0);
					readDP(DP_RDBUFF,&readWord0);
				}while((readWord0>>16)&0x1); // 当为1的时候说明没有操作完成！
				
				
				
				for( u8 i = startAddressPage; i<= endAddressPage; i++ )
				{
					writeAP( AP_CSW, 0x23000002 );   // 写32位
					writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // 擦除操作，同时设置PSIZE值为2，表示为32位操作大小！
//					if( i<12 )
//					{
						writeAP( AP_DRW, 0x0202 + (i<<3) );  // 扇区擦除，加上页号
						writeAP( AP_TAR, (u32)&F4_FLASH->CR );
						writeAP( AP_DRW, 0x0202 + (i<<3) + (1<<16) ); // 开始操作
//					}
//					else
//					{
//						writeAP( AP_DRW, 0x0202 + ((i+4)<<3) );  // 扇区擦除，加上页号
//						writeAP( AP_TAR, (u32)&F4_FLASH->CR );
//						writeAP( AP_DRW, 0x0202 + ((i+4)<<3) + (1<<16) ); // 开始操作
//					}
					
					
					writeAP( AP_CSW,0x00000002);   // 读
					u32 readWord1 = 0;
					do
					{
						writeAP(AP_TAR, (u32)&F4_FLASH->SR);
						readAP(AP_DRW,&readWord1);
						readDP(DP_RDBUFF,&readWord1);
					}while((readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
				}
				
				if( userfile.RollingCodeFunction == 0x55 ) // 使能滚码操作，只有当滚码操作使能的时候，再操作，防止访问了错误的地址！
				{
					for( u8 i = startRollingAddressPage; i<= endRollingdressPage; i++ )
					{
						writeAP( AP_CSW,0x23000002 );   // 写32位
						writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // 擦除操作，同时设置PSIZE值为2，表示为32位操作大小！
//						if( i<12 )
//						{
							writeAP( AP_DRW, 0x0202 + (i<<3) );  // 扇区擦除，加上页号
							writeAP( AP_TAR, (u32)&F4_FLASH->CR );
							writeAP( AP_DRW, 0x0202 + (i<<3) + (1<<16) ); // 开始操作
//						}
//						else
//						{
//							writeAP( AP_DRW, 0x0202 + ((i+4)<<3) );  // 扇区擦除，加上页号
//							writeAP( AP_TAR, (u32)&F4_FLASH->CR );
//							writeAP( AP_DRW, 0x0202 + ((i+4)<<3) + (1<<16) ); // 开始操作
//						}
						
						writeAP( AP_CSW,0x00000002);   // 读
						u32 readWord1 = 0;
						do
						{
							writeAP(AP_TAR, (u32)&F4_FLASH->SR);
							readAP(AP_DRW,&readWord1);
							readDP(DP_RDBUFF,&readWord1);
						}while((readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
					}
				}
				
				{ // 锁定Flash
					writeAP( AP_CSW,0x23000002 );   // 写32位
					writeAP( AP_TAR, (u32)&F4_FLASH->CR );
					writeAP( AP_DRW, 0x80000000 );
				}
			}
		}
	}
	{ // 写Flash，同时要写滚码操作区！
		
		{  // 解锁Flash操作
			writeAP( AP_CSW,0x23000002);   // 32位写访问
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0x45670123 );
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0xCDEF89AB );
			
			writeAP( AP_CSW,0x00000002);   // 读
			u32 readWord0 = 0;
			do
			{
				writeAP(AP_TAR, (u32)&F4_FLASH->SR);
				readAP(AP_DRW,&readWord0);
				readDP(DP_RDBUFF,&readWord0);
			}while((readWord0>>16)&0x1); // 当为1的时候说明没有操作完成！
		}
		{ // 使能编辑位
			writeAP( AP_TAR, (u32)&F4_FLASH->CR );
			writeAP(AP_DRW, 0x0201 );  // 32宽度，PG使能
		}
		
		{ // 写程序区
			u32 HowMany4K = 0;
			if( userfile.ProgramSize % 0x1000 != 0 )
				HowMany4K = userfile.ProgramSize/0x1000 + 1;
			else
				HowMany4K = userfile.ProgramSize/0x1000;
			
			for( u32 tmp11 = 0;tmp11<HowMany4K; tmp11++ )  // 写Flash的操作，每次操作4K字节
			{
				u32 Address = userfile.ProgramStartAddress + tmp11*0x1000;  // 起始操作地址
				W25QXX_Read( W25Q4KBuf.databuf4K , userfile.ProgramSaveAtW25QAddress + tmp11*0x1000, 0x1000); // 4k大小在16进行下，就是0x1000
				
				writeAP( AP_CSW,0x23000002 | 0x10 ); // 长度为32bit,写，且地址自动增加！
				writeAP( AP_TAR, Address );  // 地址！
				for( u32 tmp12 = 0;tmp12 <0x1000; tmp12 += 4 ) // 写入4K
				{
					{ // 实测，可以把下面的给省略掉，这样可以提升速度一个测试是：从56S - 36S！因为写单个可能根本不需要这样！但是这个暂时保留，日后防止速度上来后，影响速度！
//								writeAP( AP_CSW,0x00000002 );   // 读
//								do
//								{
//									writeAP(AP_TAR, (u32)&FLASH->SR.All);
//									readAP(AP_DRW,&readWord0);
//									readDP(DP_RDBUFF,&readWord0);
//								}while(readWord0&0x1); // 当为1的时候说明没有操作完成！
					}
					// 下面的方法，可以改为4K自动增加的，但是需要花时间，而且暂时这个下载器的速度，也不是很慢，所以暂时先不考虑，日后为了提升速度，可以增加这个！
					if( ((Address + tmp12) %0x1000) == 0 ) // 说明到了4k临界点，需要重新写地址
					{
						writeAP( AP_TAR, Address + tmp12 );  // 地址！
					}
					
//					if( (Address+tmp12)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
//					{
						writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+3] << 24) + ( W25Q4KBuf.databuf4K[tmp12+2] << 16 ) + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
//					}
//					else
//					{
//						writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
//					}
				}
				{ // 程序进度区！程序进度是4K大小！进度梯度是10%为进度！
					// 这个使用emWin应该是一个简单的问题！
					float tmp13 = (float)((float)((float)tmp11+1.0)/(float)HowMany4K);
					for( float tmp14 = 0.0;tmp14 <= (float)1.0; tmp14 += 0.1 )
					{
						if( tmp13 <= tmp14 ) // 找到第一次开始小于等于的值！
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
		{ // 写滚码区
			u32 programTimesTmp15 = 0;  // 实际已经编程次数
			u32 valueTmp16 = 0; 				// 实际写入的滚码值
			if( userfile.RollingCodeFunction == 0x55 ) // 表示需要操作滚码区
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
							programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + Tmp16;  // 实际编程次数
							break;
						}
					}
				}
				valueTmp16 = userfile.RollingCodeStartValue + ( userfile.RollingCodeStepValue* programTimesTmp15 );
				writeAP( AP_TAR, userfile.RollingCodeStartAddress );  // 地址！
//				{
//					if( (userfile.RollingCodeStartAddress)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
//					{
						writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//					}
//					else
//					{
//						writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//					}
//				}
//				{
//					writeAP( AP_TAR, userfile.RollingCodeStartAddress+2 );  // 地址！
//					{
//						if( (userfile.RollingCodeStartAddress+2)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
//						{
//							writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
//						}
//						else
//						{
//							writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//						}
//					}
//				}
			
				{ // 检验滚码区
					u32 apId = 0;
					writeDP( DP_SELECT, 0x00 );  // 访问寄存器
					writeAP( AP_CSW,0x2 | 0x10);   // 长度为32bit，同时地址自动增加
					writeAP( AP_TAR,userfile.RollingCodeStartAddress);   // 写目标地址
					readAP( AP_DRW,&apId);  // 第一次数据，不要！
					readDP( DP_RDBUFF, &apId );
					if( apId != valueTmp16 )
					{
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*11,12,RED, "    滚码校验失败     ");
						}
						else
						{
							LCD_ShowString(0,12*11,12,RED, "Roll code check failed");
						}
						returnNumber = 0x00;  // 检测错误
						return returnNumber;
					}
				}
			}
		}
		{ // 校验程序区，暂时只使用和校验！
			switch( userfile.FlashCheckWay )
			{
				case 0x00: // 不检验
					if( L_ENGLISH_VER != 0x55AA )
					{
						LCD_ShowString(0,12*11,12,RED, "     不校验FLASH     ");
					}
					else
					{
						LCD_ShowString(0,12*11,12,RED, "   not check FLASH   ");
					}
					returnNumber = 0x55AA;
					break;
				case 0x01: // 逐个检验
					break;
				case 0x02: // CRC校验（和校验） -- 日后应该只会用这种校验方式，因为其它的也没有必要性了！
					{ // 这样的校验方式有这样的要求：起始地址4字节对齐，程序大小4字节倍数！这个日后务必要保证，如果保证不了，可以从软件上解决这个问题！
						u16 sumTmp15 = 0;  // 最后的和校验程序
						writeDP( DP_SELECT, 0x00 );  // 访问寄存器
						writeAP( AP_CSW,0x2 | 0x10);   // 长度为32bit，同时地址自动增加
						u32 addressTmp16 = userfile.ProgramStartAddress;
						writeAP( AP_TAR,addressTmp16);   // 写目标地址
						
						u32 apId = 0;
						readAP( AP_DRW,&apId);  // 第一次数据，不要！
						
						for( ;addressTmp16 < ( userfile.ProgramStartAddress + userfile.ProgramSize );addressTmp16 += 4 )
						{
//									u32 apId = 0;
//									if( ( addressTmp16 %0x1000 ) == 0 )
//									{
//										writeAP( AP_TAR,addressTmp16);   // 写目标地址
//									}
//									readAP( AP_DRW,&apId);  // 关于读数据的话，每次只能读上一次的。就是有一次延时。
//									readDP( DP_RDBUFF, &apId );
//									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
							if( ( addressTmp16 %0x1000 ) == 0 )
							{
								writeAP( AP_TAR,addressTmp16);   // 写目标地址
								readAP( AP_DRW,&apId); 
							}
							readAP( AP_DRW,&apId);  // 这次数据是上次的。但是也是需要的。
							sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
						}
						if( sumTmp15 == userfile.SumValueOfProgram )
						{
							if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*11,12, BLACK,"   FLASH 校验成功   ");
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
										LCD_ShowString(0,12*11,12,BLACK, "   FLASH 校验失败    ");
									}
										else
									{
										LCD_ShowString(0,12*11,12, BLACK," FLASH Check failure ");
									}
							returnNumber = 0x1122;
						}
					}
					break;
				case 0x03: // MD5值检验
					break;
				default:
					break;
			}
		}
		{ // 写滚码，且校验滚码区！！
			// 为了使滚码操作比较方便：要求1：必须在4字节对齐地址上，必须使用小端模式！
			{ // 清除滚码地址
				
			}
			{ // 写滚码
				
			}
			{ // 校验滚码，使用每个对比！
				
			}
		}
		{ // 操作保护区！(其实这里应该加一个两个都不使能的操作！)
			switch( userfile.OptionBytesFunction )
			{
				case 0x55: // 使能选项字节，日后功能开放！
					break;
				case 0xAA: // 使能读保护
					stm32f2Protection( ENABLE,DISABLE );
					break;
				case 0x11: // 使能写保护
					stm32f2Protection( DISABLE,ENABLE );
					break;
				case 0x1A: // 使能读写保护
					stm32f2Protection( ENABLE,ENABLE );
					break;
			}
		}
		{
			writeAP( AP_CSW,0x23000002); // 写
			writeAP(AP_TAR, (u32)&F4_FLASH->CR);
			writeAP(AP_DRW, 0x80000000);  // 禁止编程和锁定
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
	u32 F4FlashMemorySize = 0;  // 在操作擦除操作和保护操作时，必须要知道其大小，否则会出现没有定义的值！
	{ // 
		{ // 检查Flash的大小。
			u32 apId = 0;
			writeDP( DP_SELECT, 0x00 );  // 访问寄存器
			writeAP( AP_CSW,0x2 );   // 长度为32bit，同时地址自动增加
			writeAP( AP_TAR,(u32)(0x1FFF7A22));   // 写目标地址
			readAP( AP_DRW,&apId);  // 第一次数据，不要！
			readDP( DP_RDBUFF, &apId );
			F4FlashMemorySize = apId>>16;
			{
				if( userfile.ChipTypePrefix == 0x20)
				{
					F4FlashMemorySize = 0;
				}
			}
		}
		{ // 检查保护区！
					OS_ERR err;
					u32 apId = 0;
					writeDP( DP_SELECT, 0x00 );  // 访问寄存器
					writeAP( AP_CSW,0x2 );   // 长度为32bit，同时地址自动增加
					writeAP( AP_TAR,(u32)&F4_FLASH->OPTCR);   // 写目标地址
					readAP( AP_DRW,&apId);  // 第一次数据，不要！
					readDP( DP_RDBUFF, &apId );
			
			
					if( ((apId>>8)&(0xFF)) == 0xAA ) // 没有激活读保护
					{
						{ // 检查写保护
							if( ((apId>>16)&(0x0FFF)) != 0x0FFF )  // 说明存在写保护
							{
								if( L_ENGLISH_VER != 0x55AA )
								{
									LCD_ShowString(0,12*10,12,RED, "      发现写保护      ");
								}
								else
								{
									LCD_ShowString(0,12*10,12,RED, "  have write protect  ");
								}
								goto dealTmp0;
							}
							
							if( ( F4FlashMemorySize > 0x400 ) && (F4FlashMemorySize != 0xFFFF)) // 大于1M的存储，实际中发现了：0xFFFF，而且一个可能的STM32F1假芯片也发现了这个问题！
							{
								writeAP( AP_TAR,(u32)&F4_FLASH->OPTCR1);
								readAP( AP_DRW,&apId);
								readDP( DP_RDBUFF, &apId );
								if( ((apId>>16)&(0x0FFF)) != 0x0FFF )  // 说明存在写保护
								{
									if( L_ENGLISH_VER != 0x55AA )
									{
										LCD_ShowString(0,12*10,12,RED, "      发现写保护      ");
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
					else if( ((apId>>8)&(0xFF)) == 0xCC )  // 不可以恢复的读保护
					{
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*10,12,RED, "         警告        ");
							LCD_ShowString(0,12*11,12,RED, "       读保护-LV2    ");
						}
						else
						{
							LCD_ShowString(0,12*10,12,RED, "       warning     ");
							LCD_ShowString(0,12*11,12,RED, "    ReadProtect-LV2  ");
						}
						return 0x00;
					}
					else  // 设置了可以清除的读保护
					{ // 设置了读保护，要清除！
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*10,12,RED, "      发现读保护      ");
						}
						else
						{
							LCD_ShowString(0,12*10,12,RED, "      read protect    ");
						}
						dealTmp0:
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*11,12,RED, "        清除中.      ");
						}
						else
						{
							LCD_ShowString(0,12*11,12,RED, "      Cleaning...    ");
						}
						{ // 清除SWD的错误！
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
							LCD_ShowString(0,12*10,12,RED, "      保护已清除     ");
							LCD_ShowString(0,12*11,12,RED, "    请断电重新下载   ");
						}
						else
						{
							LCD_ShowString(0,12*10,12,RED, " Protect cleared,Download again without power!");
						}
						return 0xFFBB;
					}
			}
	}
	
	{ // 清除Flash
		if( userfile.ChipEraseWay == 00 )  // 全擦除
		{
			writeAP( AP_CSW,0x23000002);   // 32位写访问
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0x45670123 );
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0xCDEF89AB );
			
			writeAP( AP_CSW,0x00000002);   // 读
			u32 readWord0 = 0;
			do
			{
				writeAP(AP_TAR, (u32)&F4_FLASH->SR);
				readAP(AP_DRW,&readWord0);
				readDP(DP_RDBUFF,&readWord0);
			}while((readWord0>>16)&0x1); // 当为1的时候说明没有操作完成！
			
// 注意：之前是　　 F4FlashMemorySize > 0x400 没有发现错误！
			if( F4FlashMemorySize <= 0x400 )  // 全擦除操作需要注意容量大小
			{
				writeAP( AP_CSW,0x23000002);   // 写32位
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // 擦除操作，同时设置PSIZE值为2，表示为32位操作大小！
				writeAP( AP_DRW, 0x0204 );
				
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );
				writeAP( AP_DRW, 0x0204 + ( 1<<16 )); // 开始操作
				
				
				writeAP( AP_CSW,0x00000002);   // 读
				u32 readWord1 = 0;
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord1);
					readDP(DP_RDBUFF,&readWord1);
				}while((readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
			}
			else
			{	
				writeAP( AP_CSW,0x23000002);   // 写32位
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // 擦除操作，同时设置PSIZE值为2，表示为32位操作大小！
				writeAP( AP_DRW, 0x8204 );
				
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );
				writeAP( AP_DRW, 0x8204 + ( 1<<16 )); // 开始操作
				
				
				writeAP( AP_CSW,0x00000002);   // 读
				u32 readWord1 = 0;
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord1);
					readDP(DP_RDBUFF,&readWord1);
				}while((readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
			}
			
			{ // 锁定Flash,由于没有加这个，最后导致出现了问题！
				writeAP( AP_CSW,0x23000002 );   // 写32位
				writeAP( AP_TAR, (u32)&F4_FLASH->CR );
				writeAP( AP_DRW, 0x80000000 );
			}
		}
		else	// 块擦除，分为小容量大容量和大容量，但是大容量也可以用1KB的来操作，这样可以简化，但是浪费时间，第一个版本不用太在意！
					// 除此之外，还有滚码所在的地址！
					// 注意F4的块问题。因为分配均匀，所以会比较不一样。可以考虑用表结构！
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
			
			u32 startAddressTmp0 = 0x08000000; // 起始页所在地址
			u32 endAddressTmp1 = 0x08000000;   // 结束页所在地址
			u32 startAddressPage = 0;
			u32 endAddressPage = 0;
																
			u32 startRollingAddressTmp3 = 0x08000000;
			u32 endRollingAddressTmp4 = 0x08000000;
			u32 startRollingAddressPage = 0;
			u32 endRollingdressPage = 0;
																
			for( u32 tmp5 = 0;tmp5<1024;tmp5++ ) // 1024：最大为1M大小！
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
				if( (userfile.RollingCodeStartAddress + 32 ) < ( 0x08000000 + 0x400 * tmp8 ) )  // 这个的SIZE只有32
				{
					endRollingAddressTmp4 = 0x08000000 + 0x400 * (tmp8 - 1);
					break;
				}
			}
			{ // 找扇区所在的页号
				for( u8 Temp0 = 0;Temp0<4;Temp0++ ) // 总共查找4次
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
			{ // 用SWD清除Flash操作
				writeAP( AP_CSW,0x23000002);   // 32位写访问
			
				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
				writeAP( AP_DRW, 0x45670123 );
				
				writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
				writeAP( AP_DRW, 0xCDEF89AB );
				
				writeAP( AP_CSW,0x00000002);   // 读
				u32 readWord0 = 0;
				do
				{
					writeAP(AP_TAR, (u32)&F4_FLASH->SR);
					readAP(AP_DRW,&readWord0);
					readDP(DP_RDBUFF,&readWord0);
				}while((readWord0>>16)&0x1); // 当为1的时候说明没有操作完成！
				
				
				
				for( u8 i = startAddressPage; i<= endAddressPage; i++ )
				{
					writeAP( AP_CSW,0x23000002 );   // 写32位
					writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // 擦除操作，同时设置PSIZE值为2，表示为32位操作大小！
					if( i<12 )
					{
						writeAP( AP_DRW, 0x0202 + (i<<3) );  // 扇区擦除，加上页号
						writeAP( AP_TAR, (u32)&F4_FLASH->CR );
						writeAP( AP_DRW, 0x0202 + (i<<3) + (1<<16) ); // 开始操作
					}
					else
					{
						writeAP( AP_DRW, 0x0202 + ((i+4)<<3) );  // 扇区擦除，加上页号
						writeAP( AP_TAR, (u32)&F4_FLASH->CR );
						writeAP( AP_DRW, 0x0202 + ((i+4)<<3) + (1<<16) ); // 开始操作
					}
					
					
					writeAP( AP_CSW,0x00000002);   // 读
					u32 readWord1 = 0;
					do
					{
						writeAP(AP_TAR, (u32)&F4_FLASH->SR);
						readAP(AP_DRW,&readWord1);
						readDP(DP_RDBUFF,&readWord1);
					}while((readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
				}
				
				if( userfile.RollingCodeFunction == 0x55 ) // 使能滚码操作，只有当滚码操作使能的时候，再操作，防止访问了错误的地址！
				{
					for( u8 i = startRollingAddressPage; i<= endRollingdressPage; i++ )
					{
						writeAP( AP_CSW,0x23000002 );   // 写32位
						writeAP( AP_TAR, (u32)&F4_FLASH->CR );  // 擦除操作，同时设置PSIZE值为2，表示为32位操作大小！
						if( i<12 )
						{
							writeAP( AP_DRW, 0x0202 + (i<<3) );  // 扇区擦除，加上页号
							writeAP( AP_TAR, (u32)&F4_FLASH->CR );
							writeAP( AP_DRW, 0x0202 + (i<<3) + (1<<16) ); // 开始操作
						}
						else
						{
							writeAP( AP_DRW, 0x0202 + ((i+4)<<3) );  // 扇区擦除，加上页号
							writeAP( AP_TAR, (u32)&F4_FLASH->CR );
							writeAP( AP_DRW, 0x0202 + ((i+4)<<3) + (1<<16) ); // 开始操作
						}
						
						writeAP( AP_CSW,0x00000002);   // 读
						u32 readWord1 = 0;
						do
						{
							writeAP(AP_TAR, (u32)&F4_FLASH->SR);
							readAP(AP_DRW,&readWord1);
							readDP(DP_RDBUFF,&readWord1);
						}while((readWord1>>16)&0x1); // 当为1的时候说明没有操作完成！
					}
				}
				
				{ // 锁定Flash
					writeAP( AP_CSW,0x23000002 );   // 写32位
					writeAP( AP_TAR, (u32)&F4_FLASH->CR );
					writeAP( AP_DRW, 0x80000000 );
				}
			}
		}
	}
	{ // 写Flash，同时要写滚码操作区！
		
		{  // 解锁Flash操作
			writeAP( AP_CSW,0x23000002);   // 32位写访问
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0x45670123 );
			
			writeAP( AP_TAR, (u32)&F4_FLASH->KEYR );
			writeAP( AP_DRW, 0xCDEF89AB );
			
			writeAP( AP_CSW,0x00000002);   // 读
			u32 readWord0 = 0;
			do
			{
				writeAP(AP_TAR, (u32)&F4_FLASH->SR);
				readAP(AP_DRW,&readWord0);
				readDP(DP_RDBUFF,&readWord0);
			}while((readWord0>>16)&0x1); // 当为1的时候说明没有操作完成！
		}
		{ // 使能编辑位
			writeAP( AP_TAR, (u32)&F4_FLASH->CR );
			writeAP(AP_DRW, 0x0201 );  // 32宽度，PG使能
		}
		
		{ // 写程序区
			u32 HowMany4K = 0;
			if( userfile.ProgramSize % 0x1000 != 0 )
				HowMany4K = userfile.ProgramSize/0x1000 + 1;
			else
				HowMany4K = userfile.ProgramSize/0x1000;
			
			for( u32 tmp11 = 0;tmp11<HowMany4K; tmp11++ )  // 写Flash的操作，每次操作4K字节
			{
				u32 Address = userfile.ProgramStartAddress + tmp11*0x1000;  // 起始操作地址
				W25QXX_Read( W25Q4KBuf.databuf4K , userfile.ProgramSaveAtW25QAddress + tmp11*0x1000, 0x1000); // 4k大小在16进行下，就是0x1000
				
				writeAP( AP_CSW,0x23000002 | 0x10 ); // 长度为32bit,写，且地址自动增加！
				writeAP( AP_TAR, Address );  // 地址！
				for( u32 tmp12 = 0;tmp12 <0x1000; tmp12 += 4 ) // 写入4K
				{
					{ // 实测，可以把下面的给省略掉，这样可以提升速度一个测试是：从56S - 36S！因为写单个可能根本不需要这样！但是这个暂时保留，日后防止速度上来后，影响速度！
//								writeAP( AP_CSW,0x00000002 );   // 读
//								do
//								{
//									writeAP(AP_TAR, (u32)&FLASH->SR.All);
//									readAP(AP_DRW,&readWord0);
//									readDP(DP_RDBUFF,&readWord0);
//								}while(readWord0&0x1); // 当为1的时候说明没有操作完成！
					}
					// 下面的方法，可以改为4K自动增加的，但是需要花时间，而且暂时这个下载器的速度，也不是很慢，所以暂时先不考虑，日后为了提升速度，可以增加这个！
					if( ((Address + tmp12) %0x1000) == 0 ) // 说明到了4k临界点，需要重新写地址
					{
						writeAP( AP_TAR, Address + tmp12 );  // 地址！
					}
					
//					if( (Address+tmp12)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
//					{
						writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+3] << 24) + ( W25Q4KBuf.databuf4K[tmp12+2] << 16 ) + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
//					}
//					else
//					{
//						writeAP(AP_DRW, + ( W25Q4KBuf.databuf4K[tmp12+1] << 8) + ( W25Q4KBuf.databuf4K[tmp12] ) ); 
//					}
				}
				{ // 程序进度区！程序进度是4K大小！进度梯度是10%为进度！
					// 这个使用emWin应该是一个简单的问题！
					float tmp13 = (float)((float)((float)tmp11+1.0)/(float)HowMany4K);
					for( float tmp14 = 0.0;tmp14 <= (float)1.0; tmp14 += 0.1 )
					{
						if( tmp13 <= tmp14 ) // 找到第一次开始小于等于的值！
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
		{ // 写滚码区
			u32 programTimesTmp15 = 0;  // 实际已经编程次数
			u32 valueTmp16 = 0; 				// 实际写入的滚码值
			if( userfile.RollingCodeFunction == 0x55 ) // 表示需要操作滚码区
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
							programTimesTmp15 = userfile.AlreadyProgrammedCarrayBit*(256+1) + Tmp16;  // 实际编程次数
							break;
						}
					}
				}
				valueTmp16 = userfile.RollingCodeStartValue + ( userfile.RollingCodeStepValue* programTimesTmp15 );
				writeAP( AP_TAR, userfile.RollingCodeStartAddress );  // 地址！
//				{
//					if( (userfile.RollingCodeStartAddress)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
//					{
						writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//					}
//					else
//					{
//						writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//					}
//				}
//				{
//					writeAP( AP_TAR, userfile.RollingCodeStartAddress+2 );  // 地址！
//					{
//						if( (userfile.RollingCodeStartAddress+2)%4 != 0 ) // 表示不是4的倍数，则取的16位在高位
//						{
//							writeAP(AP_DRW, + ( ((valueTmp16>>24)&0xFF) << 24) + ( ((valueTmp16>>16)&0xFF) << 16 ) ); 
//						}
//						else
//						{
//							writeAP(AP_DRW, + ( ((valueTmp16>>8)&0xFF) << 8) + ( ((valueTmp16>>0)&0xFF) ) ); 
//						}
//					}
//				}
			
				{ // 检验滚码区
					u32 apId = 0;
					writeDP( DP_SELECT, 0x00 );  // 访问寄存器
					writeAP( AP_CSW,0x2 | 0x10);   // 长度为32bit，同时地址自动增加
					writeAP( AP_TAR,userfile.RollingCodeStartAddress);   // 写目标地址
					readAP( AP_DRW,&apId);  // 第一次数据，不要！
					readDP( DP_RDBUFF, &apId );
					if( apId != valueTmp16 )
					{
						if( L_ENGLISH_VER != 0x55AA )
						{
							LCD_ShowString(0,12*11,12,RED, "    滚码校验失败     ");
						}
						else
						{
							LCD_ShowString(0,12*11,12,RED, "Roll code check failed");
						}
						returnNumber = 0x00;  // 检测错误
						return returnNumber;
					}
				}
			}
		}
		{ // 校验程序区，暂时只使用和校验！
			switch( userfile.FlashCheckWay )
			{
				case 0x00: // 不检验
					if( L_ENGLISH_VER != 0x55AA )
					{
						LCD_ShowString(0,12*11,12,RED, "     不校验FLASH     ");
					}
					else
					{
						LCD_ShowString(0,12*11,12,RED, "   not check FLASH   ");
					}
					returnNumber = 0x55AA;
					break;
				case 0x01: // 逐个检验
					break;
				case 0x02: // CRC校验（和校验） -- 日后应该只会用这种校验方式，因为其它的也没有必要性了！
					{ // 这样的校验方式有这样的要求：起始地址4字节对齐，程序大小4字节倍数！这个日后务必要保证，如果保证不了，可以从软件上解决这个问题！
						u16 sumTmp15 = 0;  // 最后的和校验程序
						writeDP( DP_SELECT, 0x00 );  // 访问寄存器
						writeAP( AP_CSW,0x2 | 0x10);   // 长度为32bit，同时地址自动增加
						u32 addressTmp16 = userfile.ProgramStartAddress;
						writeAP( AP_TAR,addressTmp16);   // 写目标地址
						
						u32 apId = 0;
						readAP( AP_DRW,&apId);  // 第一次数据，不要！
						
						for( ;addressTmp16 < ( userfile.ProgramStartAddress + userfile.ProgramSize );addressTmp16 += 4 )
						{
//									u32 apId = 0;
//									if( ( addressTmp16 %0x1000 ) == 0 )
//									{
//										writeAP( AP_TAR,addressTmp16);   // 写目标地址
//									}
//									readAP( AP_DRW,&apId);  // 关于读数据的话，每次只能读上一次的。就是有一次延时。
//									readDP( DP_RDBUFF, &apId );
//									sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
							if( ( addressTmp16 %0x1000 ) == 0 )
							{
								writeAP( AP_TAR,addressTmp16);   // 写目标地址
								readAP( AP_DRW,&apId); 
							}
							readAP( AP_DRW,&apId);  // 这次数据是上次的。但是也是需要的。
							sumTmp15 += GetSumOf16Bit((u8 *)&apId,4);
						}
						if( sumTmp15 == userfile.SumValueOfProgram )
						{
							if( L_ENGLISH_VER != 0x55AA )
							{
								LCD_ShowString(0,12*11,12, BLACK,"   FLASH 校验成功   ");
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
								LCD_ShowString(0,12*11,12,BLACK, "   FLASH 校验失败    ");
							}
								else
							{
								LCD_ShowString(0,12*11,12, BLACK," FLASH Check failure ");
							}
							returnNumber = 0x1122;
							return returnNumber;  // 此时认为已失败！返回。
						}
					}
					break;
				case 0x03: // MD5值检验
					break;
				default:
					break;
			}
		}
		{ // 写滚码，且校验滚码区！！
			// 为了使滚码操作比较方便：要求1：必须在4字节对齐地址上，必须使用小端模式！
			{ // 清除滚码地址
				
			}
			{ // 写滚码
				
			}
			{ // 校验滚码，使用每个对比！
				
			}
		}
		{ // 操作保护区！(其实这里应该加一个两个都不使能的操作！)
			switch( userfile.OptionBytesFunction )
			{
				case 0x55: // 使能选项字节，日后功能开放！
					break;
				case 0xAA: // 使能读保护
					if( userfile.ChipTypePrefix == 0x20 )
							{
								stm32f2Protection( ENABLE,DISABLE );
							}
							else
							{
								stm32f4Protection( ENABLE,DISABLE );
							}
					break;
				case 0x11: // 使能写保护
					if( userfile.ChipTypePrefix == 0x20 )
							{
								stm32f2Protection( DISABLE,ENABLE );
							}
							else
							{
								stm32f4Protection( DISABLE,ENABLE );
							}
					break;
				case 0x1A: // 使能读写保护
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
			writeAP( AP_CSW,0x23000002); // 写
			writeAP(AP_TAR, (u32)&F4_FLASH->CR);
			writeAP(AP_DRW, 0x80000000);  // 禁止编程和锁定
		}
		{
			resetAndHaltTarget( );
			runTarget( );
		}
	}
	return returnNumber;
}

/**
 *  @B 操作Flash的终极程序
 *  @Attention 当返回值为0x55AA的时候，认为操作已经成功，此时需要更新程序的下载次数！(滚码是根据已下载次数来的。)
                         0x1122的时候，表示操作失败，具体日后需要不需要定义出错码，看日后V2.0版本的升级！
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
		return 0;  // 没有下载次数了。
	}
	if( userfile.ChipTypePrefix != 0x60 )  // 表示不是STM8器件！
	{
		SWDInit();  // 在这里进行初始化操作！
		
		SWD_RESET_PIN = 0;  // 实际中的SWD ResetPin应用在什么位置比较重要。需要实际进行测试才可以完成。
		if( HARDWARE_VERSION == MINI_VERSION )
		{
			GPIOB->ODR.OutputData6_RW = 0;  // PB6
			GPIOB->ODR.OutputData1_RW = 0;  // PB1
		}
		
		
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_NON_STRICT,&err);  // 延时20ms使复位更加稳定。
		dpID = initDp( );
		if( dpID == 0x00 || dpID == 0xFFFFFFFF )
		{
			if( L_ENGLISH_VER != 0x55AA )
			{
				LCD_ShowString(0,12+12*9,12,RED, " SWD 目标 dpID 错误  "  );
				LCD_ShowString(0,12+12*10,12,RED,"   请检查硬件连接    " );
			}
			else
			{
				LCD_ShowString(0,12+12*9,12,RED, "    SWD dpID error   "  );
				LCD_ShowString(0,12+12*10,12,RED,"    check hardware   " );
			}
			return 0x00;  // 正确的芯片是应该有ID的，否则认为芯片在开始时候就连接错误！
		}
		
		apID = readApID( );
		if( apID == 0x00 || apID == 0xFFFFFFFF )
		{
			if( L_ENGLISH_VER != 0x55AA )
			{
				LCD_ShowString(0,12+12*9,12,RED, " SWD 目标 apID 错误  " );
				LCD_ShowString(0,12+12*10,12,RED,"   请检查硬件连接    " );
			}
			else
			{
				LCD_ShowString(0,12+12*9,12,RED, "    SWD apID error   "  );
				LCD_ShowString(0,12+12*10,12,RED,"    check hardware   " );
			}
			return 0x00;
		}
		
		haltTarget();   // 实测发现：使用这一句或者：resetAndHaltTarget和最终的结果没有关系！根据测试发现：在复位之中，使initDp和readApID也可以达到停止MCU的功能。这个比较纠结。需要实测！
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_NON_STRICT,&err);  // 延时20ms使复位更加稳定。
		
		if( HARDWARE_VERSION == MINI_VERSION )
		{
			GPIOB->ODR.OutputData6_RW = 1;  // PB6
			GPIOB->ODR.OutputData1_RW = 1;  // PB1
		}
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_NON_STRICT,&err);  // 延时20ms使复位更加稳定。
		
		
		SWD_RESET_PIN = 1;  // 这个时候可以恢复了。
		
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_NON_STRICT,&err);  // 延时20ms使复位更加稳定。
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
		case 0x00: // F0系列
			{ // 检查是否存在SWD功能
				returnNumber = DownloadFlashOfSTM32F0xx( userfile );
				if( returnNumber == 0xFFBB && forDownloaderAgain == 1 )
				{
					forDownloaderAgain = 0;
					goto reDownloader;
				}
			}
			break;
		case 0x10:  // 本函数的暂时操作为STM32F1系列函数！
//		{
//			returnNumber = DownloadFlashOfSTM32F1xx( userfile );
//		}
		
		{ // 检查是否存在SWD功能
				returnNumber = DownloadFlashOfSTM32F1xx( userfile );
				if( returnNumber == 0xFFBB && forDownloaderAgain == 1 )
				{
					forDownloaderAgain = 0;
					goto reDownloader;
				}
			}
			break;
//		case 0x20:  // F2系列
//			{ // 检查是否存在SWD功能
//				returnNumber = DownloadFlashOfSTM32F2xx( userfile );  // 根据发现：应该是擦除出错，需要检查和验证！！
//			}
//			if( returnNumber == 0xFFBB && forDownloaderAgain == 1 )
//			{
//				forDownloaderAgain = 0;
//				goto reDownloader;
//			}
//			break;
		case 0x30:  // F3系列
		returnNumber = DownloadFlashOfSTM32F3xx( userfile );
			break;
		case 0x20:  // F2系列
		case 0x40:  // F4系列
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
			OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_NON_STRICT,&err);  // 由于添加了蜂鸣器，且下面的关闭中断，在下载STM8的时候，用单击，会导致下载成功后蜂鸣器才作用，导致会响两声！这个100ms延时正好！！
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
	return returnNumber;  // 返回指示
}


