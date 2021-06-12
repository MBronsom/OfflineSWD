#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "usb_lib.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>
 
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void usart_sendCommand(u8 *buf,u16 size);
void usart_receive_parse(u8 *shell_string);

#endif	   
