#include <stm32f10x.h>
#include "gpio.h"
#include "stm32f10x_tim.h"

extern void Servo_TIM3_PWM_Init(void);
extern void Servo_Move(u8 angle);
