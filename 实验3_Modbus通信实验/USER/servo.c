#include "servo.h"
// TIM3 PWM 部分初始化
// PWM 输出初始化
// PA6：TIM3_CH1
void Servo_TIM3_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // 使能定时器 3 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能 GPIOA 时钟
    // 设置 GPIOA.6 为复用输出功能,输出 TIM3 CH1 的 PWM 脉冲波形
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // TIM_CH1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // 初始化 GPIO
    // 初始化 TIM3 72MHz / ((199 + 1)(7199 + 1)) = 50Hz
    TIM_TimeBaseStructure.TIM_Period = 199;
    // 设置自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = 7199;
    // 设置预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    // 设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    // TIM 向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    // 初始化 TIM3 的时间基数
    // 初始化 TIM3 Channel1 PWM 模式
    // 选择定时器模式:TIM 脉冲宽度调制模式 1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    // 比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    // 输出极性:TIM 输出比较极性高
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    // 根据指定的参数初始化外设 TIM3 OC1
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    // 使能 TIM3 在 CCR1 上的预装载寄存器
    TIM_Cmd(TIM3, ENABLE);
    // 使能 TIM3
}
// angle ：舵机角度 0~180
// 0.5ms—>0 度；1.0ms—>45 度；1.5ms—>90 度；2.0ms—>135 度；2.5ms—>180 度
void Servo_Move(u8 angle)
{
    // 函数占空比设置值：199（寄存器重装载值）+1 对应 20ms，则 4+1 对应 0.5ms，通过公式对输入角度进行转换
    TIM_SetCompare1(TIM3, 4 + 5 * (angle / 45.0));
    // 设置占空比
}
