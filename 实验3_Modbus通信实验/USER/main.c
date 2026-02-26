#include "servo.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"

// 舵机与通信参数定义
#define SERVO_ADDR 0x98		  // 舵机地址码
#define SERVO_REG_ADDR 0x2200 // 舵机指令寄存器
#define MODBUS_RX_MAX_LEN 8	  // 最大接收长度
#define ANGLE_MIN 0			  // 最小角度
#define ANGLE_MAX 180		  // 最大角度

// 全局变量
uint8_t modbus_rx_buf[MODBUS_RX_MAX_LEN]; // 接收缓冲区
volatile uint8_t rx_cnt = 0;			  // 接收计数（加 volatile）
uint8_t target_angle = 90;				  // 目标角度(默认90度)

// 函数声明
void MODBUS_Send_Reply(uint8_t func_code);
void Clear_Modbus_Buffer(void);

// Delay
void Delay(u32 count)
{
	u32 i = 0;
	for (; i < count; i++)
		;
	// 通过循环来实现延时
}

// CRC16校验计算(MODBUS-RTU协议)
uint16_t CRC16_Calculate(uint8_t *buf, uint8_t len)
{
	uint16_t crc = 0xFFFF;
	uint8_t i, j;
	for (i = 0; i < len; i++)
	{
		crc ^= buf[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 0x0001)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	return crc;
}

// RS485初始化(控制引脚PA1)
void RS485_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;		 // 485方向控制引脚PA1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_Pin_1); // 初始化为接收模式(低电平)
}

// USART2初始化(9600 8N1)
void USART2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	// 配置TX引脚(PA2)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 配置RX引脚(PA3)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 配置USART2
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	// 配置接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);
}

// 清除接收缓冲区
void Clear_Modbus_Buffer(void)
{
	uint8_t i; // 声明循环变量在函数开始处
	rx_cnt = 0;
	for (i = 0; i < MODBUS_RX_MAX_LEN; i++)
	{
		modbus_rx_buf[i] = 0;
	}
}

uint8_t current_rx_cnt;
// USART2中断服务函数(接收数据)
void USART2_IRQHandler(void)
{
	// uint8_t rx_data; // 临时存储接收的数据
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		// rx_data = USART_ReceiveData(USART2); // 先读取数据

		// // 关键判断：只有第一个字节是0x98时，才允许写入缓冲区
		// if (rx_cnt == 0)
		// {
		// 	// 第一个字节必须是舵机地址0x98，否则不接收
		// 	if (rx_data != SERVO_ADDR)
		// 	{
		// 		// 地址不符，清空计数，忽略该数据
		// 		rx_cnt = 0;
		// 	}
		// 	else
		// 	{
		// 		// 地址正确，写入第一个字节
		// 		modbus_rx_buf[rx_cnt++] = rx_data;
		// 	}
		// }
		// else
		// {
		// 	// 非第一个字节，且缓冲区未满时继续接收
		// 	if (rx_cnt < MODBUS_RX_MAX_LEN)
		// 	{
		// 		modbus_rx_buf[rx_cnt++] = rx_data;
		// 	}
		// 	else
		// 	{
		// 		// 缓冲区满，清空等待下一帧
		// 		Clear_Modbus_Buffer();
		// 	}
		// }
		// USART_ClearITPendingBit(USART2, USART_IT_RXNE); // 清除中断标志

		if (rx_cnt < MODBUS_RX_MAX_LEN)
		{
			modbus_rx_buf[rx_cnt++] = USART_ReceiveData(USART2);
		}
		else
		{
			Clear_Modbus_Buffer(); // 缓冲区满则清空并等待下一帧
		}
		USART_ClearITPendingBit(USART2, USART_IT_RXNE); // 清除中断标志
	}
	else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		current_rx_cnt = rx_cnt;
		rx_cnt = 0;
		USART_ClearITPendingBit(USART2, USART_IT_IDLE); // 清除中断标志
	}
	USART_ReceiveData(USART2);
}

// MODBUS指令解析
void MODBUS_Parse_Command(void)
{
	// 所有局部变量声明放在函数开头（符合C89标准）

	uint16_t received_crc;
	uint16_t calculated_crc;
	uint8_t func_code;
	uint16_t reg_addr;

	// 校验地址
	if (modbus_rx_buf[0] != SERVO_ADDR)
	{
		Clear_Modbus_Buffer();
		return;
	}

	// 至少需要地址+功能码+两个字节寄存器地址+两个字节CRC才能校验
	if (current_rx_cnt < 6)
	{
		return; // 不清空缓冲区，等待更多数据
	}

	// 校验CRC
	received_crc = (modbus_rx_buf[current_rx_cnt - 1] << 8) | modbus_rx_buf[current_rx_cnt - 2];
	calculated_crc = CRC16_Calculate(modbus_rx_buf, current_rx_cnt - 2);

	if (received_crc != calculated_crc)
	{
		Clear_Modbus_Buffer();
		return;
	}

	func_code = modbus_rx_buf[1];
	reg_addr = (modbus_rx_buf[2] << 8) | modbus_rx_buf[3];

	// 处理写寄存器指令(0x06)
	if (func_code == 0x06 && reg_addr == SERVO_REG_ADDR && current_rx_cnt >= 8)
	{
		target_angle = modbus_rx_buf[5];
		// 限制角度范围
		if (target_angle < ANGLE_MIN)
			target_angle = ANGLE_MIN;
		if (target_angle > ANGLE_MAX)
			target_angle = ANGLE_MAX;
		Servo_Move(target_angle);
		MODBUS_Send_Reply(0x06);
		Clear_Modbus_Buffer();
	}
	// 处理读寄存器指令(0x03)
	else if (func_code == 0x03 && reg_addr == SERVO_REG_ADDR && current_rx_cnt >= 6)
	{
		MODBUS_Send_Reply(0x03);
		Clear_Modbus_Buffer();
	}
	else
	{
		// 如果数据不完整，不清空缓冲区，等待更多数据
		return;
	}
}

// 发送MODBUS应答
void MODBUS_Send_Reply(uint8_t func_code)
{
	uint8_t tx_buf[8];
	uint8_t tx_len = 0;
	uint16_t crc;
	uint8_t i;

	tx_buf[0] = SERVO_ADDR;
	tx_buf[1] = func_code;

	if (func_code == 0x03)
	{
		// 读指令应答: 地址+功能码+数据长度+数据(2字节)+CRC
		tx_buf[2] = 0x02;		  // 数据长度
		tx_buf[3] = 0x00;		  // 角度高8位
		tx_buf[4] = target_angle; // 角度低8位
		tx_len = 5;
	}
	else if (func_code == 0x06)
	{
		// 写指令应答: 地址+功能码+寄存器地址+数据+CRC
		tx_buf[2] = (SERVO_REG_ADDR >> 8) & 0xFF; // 寄存器高8位
		tx_buf[3] = SERVO_REG_ADDR & 0xFF;		  // 寄存器低8位
		tx_buf[4] = 0x00;						  // 数据高8位
		tx_buf[5] = target_angle;				  // 数据低8位
		tx_len = 6;
	}

	// 计算CRC
	crc = CRC16_Calculate(tx_buf, tx_len);
	tx_buf[tx_len++] = crc & 0xFF;		  // CRC低8位
	tx_buf[tx_len++] = (crc >> 8) & 0xFF; // CRC高8位

	// 关闭接收中断防止冲突
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);

	// 切换为发送模式
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
	Delay(100); // 确保方向切换完成

	// 发送数据
	for (i = 0; i < tx_len; i++)
	{
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
			;
		USART_SendData(USART2, tx_buf[i]);
	}

	// 等待发送完成
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
		;
	Delay(100);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1); // 切换回接收模式

	// 重新开启接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

int main(void)
{
	// 初始化外设
	RS485_IO_Init();
	USART2_Init();
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	Servo_TIM3_PWM_Init();

	Servo_Move(90); // 初始位置

	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	
	while(1)
	{
	// 只有在接收到足够数据时才处理
	if (current_rx_cnt == 8) // 至少需要地址+功能码+寄存器地址+部分CRC
	{
		MODBUS_Parse_Command();
		current_rx_cnt = 0;
	}
}
}
