#include "key.h"

/***********************输入口初始化************************/
void Key_Init(void)
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB,ENABLE);//使能PORTC 时钟
	
	GPIO_InitStructure.GPIO_Pin  = SELECT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(SELECT_GROUP, &GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin  = OK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(OK_GROUP, &GPIO_InitStructure);//初始化
}
