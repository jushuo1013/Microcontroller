#include"stm32f10x_conf.h"
#include"gpio.h"


void led_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//定义初始化结构体
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	GPIO_InitStruct.GPIO_Pin   =   (GPIO_Pin_4)|(GPIO_Pin_5)|(GPIO_Pin_6)|(GPIO_Pin_7);//选择LED端口
	GPIO_InitStruct.GPIO_Speed =   GPIO_Speed_50MHz;//设置IO口速度
	GPIO_InitStruct.GPIO_Mode  =   GPIO_Mode_Out_PP;//配置模式为输出
	GPIO_Init(GPIOB, &GPIO_InitStruct); 	//初始化GPIOB的参数为以上结构体
}

void LEDOn(uint8_t GPIO_PIN)
{
	GPIO_ResetBits(GPIOB, (1<<GPIO_PIN));
}

void LEDOff(uint8_t GPIO_PIN)
{
	GPIO_SetBits(GPIOB, (1<<GPIO_PIN));
}
