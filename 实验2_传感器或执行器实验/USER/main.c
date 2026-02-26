#include"stm32f10x_conf.h"
#include"gpio.h"
#include <stdint.h>

void Servo_TIM3_PWM_Init(void)
{
//初始化TIM3
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//配置GPIOA.6为输出TIM 3 CH1的PWM输出
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // CH1
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &GPIO_InitStructure); 
//72MHz/((199+1)(7199+1))=50Hz对T=20ms
TIM_TimeBaseStructure.TIM_Period = 199; 
TIM_TimeBaseStructure.TIM_Prescaler = 7199; 
TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//PWM1计数器小于CCR值输出高电平，大于CCR值输出低电平
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
TIM_OC1Init(TIM3, &TIM_OCInitStructure); 
TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
TIM_Cmd(TIM3, ENABLE); 
}

//angle/90+0.5=ms
void Servo_Move(u8 angle)
{
TIM_SetCompare1(TIM3, 5-1 + angle/9); 
}

void Delay(u32 count)
{
u32 i=0;
for(;i<count;i++);
}

int main(void)
{
u16 i=0;
Servo_TIM3_PWM_Init(); 
while(1)
{
for(;i<180;i+=45) 
{
Servo_Move(i);
Delay(9000000);
}
for(;i>0;i-=45)
{
Servo_Move(i);
Delay(9000000); 
}
}
}
