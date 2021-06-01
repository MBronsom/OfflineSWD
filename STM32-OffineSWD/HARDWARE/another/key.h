#ifndef _KEY_H_
#define _KEY_H_

/***********************ͷ�ļ�����************************/
#include "stm32f10x.h"
#include "sys.h"

/***********************IO�궨��************************/

#define OK_GROUP GPIOC
#define SELECT_GROUP GPIOB

#define OK_PIN GPIO_Pin_0
#define SELECT_PIN GPIO_Pin_11


/***********************���ƺ궨��************************/

#define OK 		PCin(0)
#define SELECT			PBin(11)

/***********************��������************************/

void Key_Init(void);

#endif /*_KEY_H_*/
