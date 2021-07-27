#ifndef __LED_H__
#define __LED_H__
#include "stdint.h"


#define  ALWAYS_BRIGHT                          0xFFFF


#define  LED1_PIN                               GPIO_Pin_5   
#define  LED1_Port                              GPIOB 
#define  LED1_ON					            GPIO_SetBits(LED1_Port, LED1_PIN)	
#define  LED1_OFF					            GPIO_ResetBits(LED1_Port, LED1_PIN)

#define  LED2_PIN                               GPIO_Pin_4   
#define  LED2_Port                              GPIOB 
#define  LED2_ON					            GPIO_SetBits(LED2_Port, LED2_PIN)	
#define  LED2_OFF					            GPIO_ResetBits(LED2_Port, LED2_PIN)

#define  LED3_PIN                               GPIO_Pin_3   
#define  LED3_Port                              GPIOB 
#define  LED3_ON					            GPIO_SetBits(LED3_Port, LED3_PIN)	
#define  LED3_OFF					            GPIO_ResetBits(LED3_Port, LED3_PIN)

typedef enum LED_STATUS {
    LED_Dark,
    LED_Bright,
}LED_STT_ENUM;

typedef enum LED_NUM {
    NO_LED,
    LED1,
    LED2,
    LED3,
    LED_END,
}LED_NUM_ENUM;

typedef struct  led_ctrl LED_CTRL; 
struct led_ctrl {
    LED_NUM_ENUM LED_NUM;
    LED_STT_ENUM LED_status;
    uint16_t led_work_times;                    //单位是以ms为计时单位，当>1000时为常亮
    void (*p_led_ctrl)(LED_CTRL LED_CTRL_STR);
};
extern LED_CTRL led_ctrl_struct;                //led的状态有充电器的状态绝定

#endif // !1

