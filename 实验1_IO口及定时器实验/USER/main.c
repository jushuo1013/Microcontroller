#include"stm32f10x_conf.h"
#include"gpio.h"
#include <stdint.h>

/*void Delay(unsigned int i);

void Delay(unsigned int i)
{
unsigned int j,k;
for(j=0;j<i;j++)
for(k=0;k<10000;k++);
}

int main(void)
{
	RCC_Configuration();
	led_gpio_init();
	while(1)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_4);
    Delay(500);
		GPIO_SetBits(GPIOB,GPIO_Pin_4);
		Delay(500);
		
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);
    Delay(500);
		GPIO_SetBits(GPIOB,GPIO_Pin_5);
		Delay(500);
		
		GPIO_ResetBits(GPIOB,GPIO_Pin_6);
    Delay(500);
		GPIO_SetBits(GPIOB,GPIO_Pin_6);
		Delay(500);
		
		GPIO_ResetBits(GPIOB,GPIO_Pin_7);
    Delay(500);
		GPIO_SetBits(GPIOB,GPIO_Pin_7);
		Delay(500);
	}
}*/

void led_set(uint8_t led_num, int state)
{
    if(state)
          LEDOn(led_num);
    else
        	LEDOff(led_num);    
}

//TIM2每1秒产生一次中断
void TIM2_Configuration(void)
{
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
 TIM_DeInit(TIM2);
 TIM_TimeBaseStructure.TIM_Prescaler= (7200 - 1);
 //时钟预分频数7200即单位时间为7200/72M=0.1ms
 TIM_TimeBaseStructure.TIM_Period=(5000-1);
 //累计TIM_Period个时钟后产生一个更新或者中断即周期为5000*0.1ms=0.5s
 TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 TIM_ClearFlag(TIM2, TIM_FLAG_Update);
 TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
 TIM_Cmd(TIM2, ENABLE);
}

//TIM2中断优先级配置
void TIM2_NVIC_Configuration(void) 
{
 NVIC_InitTypeDef NVIC_InitStructure;
 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 //设置响应优先级为0,数值越小优先级最高
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);
}

//中断服务程序
void TIM2_IRQHandler(void)
{
 if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET )
 {
  static int led_state = 0;//静态局部变量,只初始化一次
	led_state++; //状态加1
  switch (led_state)
  {
   case 1: //状态1:灭掉User1,点亮Status
   led_set(4,1);
   break;
	 case 2: //状态2:灭掉User1,灭掉Status
   led_set(4,0);
   break;
   case 3: //状态3:灭掉Status,点亮RS485-R
   led_set(5,1);
	 break;
	 case 4: //状态4:灭掉Status,灭掉RS485-R
   led_set(5,0);
   break;
   case 5: //状态5:灭掉RS485-R,点亮RS485-T
   led_set(6,1);
   break;
	 case 6: //状态6:灭掉RS485-R,灭掉RS485-T
   led_set(6,0);
   break;
   case 7: //状态7:灭掉RS485-T,点亮User1
   led_set(7,1);
   break;
	 case 8: //状态8:灭掉RS485-T,灭掉User1
   led_set(7,0);
   led_state=0; //led_state置0,下个1s时间到后可再次进入状态1
   break;
   default:
   break;
  }
  TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);//清除中断标志位
 }
}

int main(void)
{
	RCC_Configuration();
	led_gpio_init();
	//gpio_init();//GPIO初始化
  //led_init(); //LED初始化
 TIM2_Configuration(); //TIM2定时配置
 TIM2_NVIC_Configuration(); //TIM2中断优先级配置
 while(1)
 {
  //主循环中什么都没有,一切在中断服务程序中完成
 }
}
