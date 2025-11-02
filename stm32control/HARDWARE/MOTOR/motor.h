#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"

#define AIN1	PAout(15)	
#define AIN2	PBout(3)

#define STBY	PBout(5)

#define BIN1	PBout(8)
#define BIN2	PBout(9)


#define		PWMB 		TIM4->CCR1
#define 	PWMA		TIM4->CCR2

void Motor_Init(void);

#endif
