#ifndef __GPIO_H__
#define __GPIO_H__
#include "stdint.h"

/* 测试使用开关控制信号 */
#define  GPIO_Pin_Key                           GPIO_Pin_1   
#define  GPIO_Port_Key                          GPIOB 
#define  KeyIn					                GPIO_ReadInputDataBit(GPIO_Port_Key, GPIO_Pin_Key)	 

#define  GPIO_Pin_ACSingal                      GPIO_Pin_2   
#define  GPIO_Port_ACSingal                     GPIOB 
#define  ACSingal_In()					        GPIO_ReadInputDataBit(GPIO_Port_ACSingal, GPIO_Pin_ACSingal)	



#define  GPIO_FAULT_OUT                         GPIO_Pin_6   
#define  GPIO_Port_FAULT                        GPIOB 
#define  RESET_FAULT					        GPIO_SetBits(GPIO_Port_FAULT, GPIO_FAULT_OUT)
#define  FAULT_Protect					        GPIO_ResetBits(GPIO_Port_FAULT, GPIO_FAULT_OUT)

#define  Relay1_Pin                             GPIO_Pin_9   
#define  Relay1_Port                            GPIOB 
#define  Relay1_ON    					        GPIO_SetBits(Relay1_Port, Relay1_Pin)
#define  Relay1_OFF					            GPIO_ResetBits(Relay1_Port, Relay1_Pin)

#define  Relay2_Pin                             GPIO_Pin_8   
#define  Relay2_Port                            GPIOB 
#define  Relay2_ON    					        GPIO_SetBits(Relay2_Port, Relay2_Pin)
#define  Relay2_OFF					            GPIO_ResetBits(Relay2_Port, Relay2_Pin)

#define  GPIO_LED3                              GPIO_Pin_3   
#define  GPIO_Port_LED3                         GPIOB 
#define  LED3_ON    					        GPIO_SetBits(GPIO_Port_LED3, GPIO_LED3)
#define  LED3_OFF					            GPIO_ResetBits(GPIO_Port_LED3, GPIO_LED3)

#define  GPIO_Pin_POWER_ON                      GPIO_Pin_12										//20210511用于控制电源输出
#define  GPIO_Port_POWER_ON                     GPIOB


#define BUTTON_FILTER_TIME 	10  //单位为ms
#define BUTTON_LONG_TIME 	1000		// 持续1秒，认为长按事件

void key_init(void);

/**
 * @brief define Button_key initialize function  
 * 
 */
typedef struct
{
	uint8_t     (*IsKeyDownFunc)(void); 		        // check function 
	uint8_t 	wFilterCount;							// 滤波器计数器 
	uint8_t 	wFilterTime;							// 滤波时间(最大255,表示255ms) 
	uint8_t	    wLongCount;								// 长按计数器 
	uint16_t	wLongTime;								// 长按键时间, 0表示不检测长按 
	uint8_t  	byState;								// 按键当前状态（按下还是弹起） 	
	uint8_t 	byKeyCodeUp;							// 按键弹起的键值代码, 0表示不检测按键弹起 	
	uint8_t 	byKeyCodeDown;						    // 按键按下的键值代码, 0表示不检测按键按下 
	uint8_t 	byKeyCodeLong;						    // 按键长按的键值代码, 0表示不检测长按 	
}Button_t;


/**
 * @brief define button enum
 * 
 */
typedef enum
{
	KEY_NONE,
	KEY_DOWN,									// KEY键按下 
	KEY_UP,									    // KEY键弹起 
	KEY_LONG,									// KEY键长按 
}Key_em;


typedef enum
{
	Closed,
	Opened,
}relay_ctrl_em;

typedef enum
{
	NO_Channel,
	Channel1,
    Channel2,
}channel_em;

//定义一个同步信号控制变量
extern uint16_t period_signl_recd;
extern uint16_t signl_timeout_flag;

void mcu_gpio_init(void);

void ButtonProj(void);
/* 使能电源的输出 */
void power_on_enabled(void);
/* 关闭电源的输出 */
void power_on_disenabled(void);

void V_OVER_Protect(void);

void Channel_ctrl(channel_em channel, relay_ctrl_em ctrl_mdl);





#endif


