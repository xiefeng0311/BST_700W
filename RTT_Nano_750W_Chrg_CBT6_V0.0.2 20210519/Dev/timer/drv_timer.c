#include "drv_time.h"
#include "GPIO.h"
#include "rtdef.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"


static uint16_t times_count = 0;


void Time3_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* The MAX frequency of APB1 BUS is 36MHz */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);                //enable TIM3 clock

    /* Initialize TIM3 */
	TIM_DeInit(TIM3);													//重新将Timer设置为缺省值
	TIM_InternalClockConfig(TIM3);
    //TIM_TimeBaseStructure.TIM_Period = arr;                             //set TIMx_ARR
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                          //set prescaler
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;             //set clock
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;         //set counter direction
    TIM_TimeBaseStructure.TIM_Period = (arr<2)?1:arr-1;		            //设置计数溢出大小，每计period个数就产生一个更新事件
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                     //initialize TIM3 
    //
    TIM_ARRPreloadConfig(TIM3, DISABLE);					            //禁止ARR预装载缓冲器

    /* Configure NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;                     //TIM3 global Interrupt 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;           //
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		            // 响应式中断优先级设置为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                     //
    NVIC_Init(&NVIC_InitStructure);                                     //
    
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);					            //清除溢出中断标志
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);                          //TIM3 configure as UP
	//TIM_Cmd(TIM3, ENABLE);
}


void Time3_Init_interfaces(void)
{
    /* Initialize it to a 1ms timer, TIM3 is Configured to 1 ms*/
    Time3_Init(1000, 35);               
}

INIT_DEVICE_EXPORT(Time3_Init_interfaces);


void Time3_Start(void)
{
    TIM_Cmd(TIM3, ENABLE);
}

void Time3_Stop(void)
{
    TIM_Cmd(TIM3, DISABLE);
}

//Tim3 用于同步信号超5ms保护
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {	
        if (ACSingal_In() == 1) {
            times_count++;
            if (times_count >= 5) {
                power_on_disenabled();
                Time3_Stop();
                times_count = 0;
            }
        } else {
            times_count = 0;
            Time3_Stop();
        }
        TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update); 
	}
}

