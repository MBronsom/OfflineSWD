#include "key.h"

/***********************����ڳ�ʼ��************************/
void Key_Init(void)
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB,ENABLE);//ʹ��PORTC ʱ��
	
	GPIO_InitStructure.GPIO_Pin  = SELECT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(SELECT_GROUP, &GPIO_InitStructure);//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin  = OK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(OK_GROUP, &GPIO_InitStructure);//��ʼ��
}
