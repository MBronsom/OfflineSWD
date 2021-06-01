#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "usb_lib.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>
 
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void usart_sendCommand(u8 *buf,u16 size);
void usart_receive_parse(u8 *shell_string);

#endif	   
