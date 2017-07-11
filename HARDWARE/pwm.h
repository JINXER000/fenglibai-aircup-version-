#ifndef _PWM_H
#define _PWM_H
#include "sys.h"
#define PWM1  TIM4->CCR1
#define PWM2  TIM4->CCR2
#define PWM3  TIM2->CCR3
#define PWM4 TIM2->CCR4

#define PWM5 TIM3->CCR1
#define PWM6 TIM3->CCR2
#define PWM7 TIM3->CCR3
#define PWM8 TIM3->CCR4

void TIM4_PWM_Init(int psc,int prd);
void TIM3_PWM_Init (int psc,int prd);
void TIM2_PWM_Init (int psc,int prd);
#endif
