#ifndef _KEY_H_
#define _KEY_H_

/***********************头文件定义************************/
#include "stm32f10x.h"
#include "sys.h"

/***********************IO宏定义************************/

#define OK_GROUP GPIOC
#define SELECT_GROUP GPIOB

#define OK_PIN GPIO_Pin_0
#define SELECT_PIN GPIO_Pin_11


/***********************控制宏定义************************/

#define OK 		PCin(0)
#define SELECT			PBin(11)

/***********************函数定义************************/

void Key_Init(void);

#endif /*_KEY_H_*/
