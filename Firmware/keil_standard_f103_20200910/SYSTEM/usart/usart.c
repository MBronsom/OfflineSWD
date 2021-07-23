#include "usart.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定向fputc函数
//printf的输出，指向fputc，由fputc输出到串口
//这里使用串口1(USART1)输出printf信息
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//等待上一次串口数据发送完成  
	USART1->DR = (u8) ch;      	//写DR,串口1将发送数据
	return ch;
}
#endif 
//end
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
void uart1_dma_enable(DMA_Channel_TypeDef*DMA_CHx)
{
    DMA_Cmd(DMA_CHx, DISABLE );
    DMA_SetCurrDataCounter(DMA_CHx,USART_REC_LEN);
    DMA_Cmd(DMA_CHx, ENABLE);
}

//////////////////////////////////////////////////////////////////

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 USART_TX_BUF[USART_REC_LEN];     //发送缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	
u8 uart1_rev_len; //接受字符总长度
char usart_rx_data[60];
  
//DMA发送数据
void usart_sendCommand(u8 *buf,u16 size)
{
	memset(USART_TX_BUF,0,100);
	memcpy(USART_TX_BUF,buf,size);
	
	DMA_Cmd(DMA1_Channel4,DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel4,size);
	DMA_Cmd(DMA1_Channel4,ENABLE);
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        /* Without this, the interrupt cannot be cleared and continues into the interrupt */
        USART_ReceiveData(USART1);
        uart1_rev_len = USART_REC_LEN - DMA_GetCurrDataCounter(DMA1_Channel5);
			
        if(uart1_rev_len != 0)
        {
            /* Call the parse function */
      			usart_receive_parse(USART_RX_BUF);
            memset(USART_RX_BUF,0,sizeof(USART_RX_BUF));
        }
        /* Clear the interrupt and reset DMA */
        USART_ClearITPendingBit(USART1,USART_IT_IDLE);
        uart1_dma_enable(DMA1_Channel5);
    }
} 
#endif

//配置串口1波特率
void usart_config(u32 baud)
{
	USART_InitTypeDef USART_InitStructure;
	USART_Cmd(USART1, DISABLE);
	USART_InitStructure.USART_BaudRate =baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

//初始化IO 串口1
//bound:波特率 
void uart_init(u32 baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    /* Enable RCC clock for USART1,GPIOA,DMA1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* Initialization GPIOA9 GPIOA10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Data format :1:8:1, no parity check, no hardware flow control */
    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    /* Enable USART interrupts, mainly for idle interrupts */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=USART_Parity_No;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART_StopBits_1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Initializes USART1 to enable USART, USART idle interrupts and USART RX DMA */
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Rx|USART_DMAReq_Tx,ENABLE);
    USART_Cmd(USART1, ENABLE);

    /* Initializes DMA and enables it */
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART_RX_BUF;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = USART_REC_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel5, ENABLE);
		
		DMA_Cmd(DMA1_Channel4,DISABLE);                                     //close DMA Channel
    DMA_DeInit(DMA1_Channel4);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);  //(uint32_t)(&USART1->DR)
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART_TX_BUF;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = USART_REC_LEN; 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 

    DMA_Init(DMA1_Channel4,&DMA_InitStructure);
    DMA_ClearFlag(DMA1_FLAG_GL4);  // clear all DMA flags
    //DMA_Cmd(DMA1_Channel4,ENABLE); // close DMA Channel
    DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);  //open DMA send inttrupt
}

extern u8 debugMode;
void usart_receive_parse(u8 *shell_string)
{
	UserToPMABufferCopy((unsigned char *)shell_string, ENDP4_TXADDR,  uart1_rev_len);
  SetEPTxCount(ENDP4, uart1_rev_len);
  SetEPTxValid(ENDP4);
	if(debugMode){
		OLED_ShowString(0,-1,"                     ",1,1);
		OLED_ShowString(0,0,"                     ",1,1);
		OLED_ShowString(0,1,"                     ",1,1);
		OLED_ShowString(0,-1,shell_string,1,1);
	}
}
