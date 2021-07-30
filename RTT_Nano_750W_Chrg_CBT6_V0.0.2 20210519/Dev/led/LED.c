#include "LED.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
//#include "board.h"
#include "rtthread.h"





void led_ctrl_function(LED_CTRL LED_CTRL_STR)
{
    if (LED_CTRL_STR.LED_NUM == LED1) {
        if (LED_CTRL_STR.LED_status == LED_Dark) {
            LED1_OFF;
        } else if (LED_CTRL_STR.LED_status == LED_Bright) {
            if (LED_CTRL_STR.led_work_times >= ALWAYS_BRIGHT) {
                LED1_ON;
            } else {
                LED1_ON;
                rt_thread_mdelay(LED_CTRL_STR.led_work_times);
                LED1_OFF;
                rt_thread_mdelay(LED_CTRL_STR.led_work_times);
            }
        }
    } else if (LED_CTRL_STR.LED_NUM == LED2) {
        if (LED_CTRL_STR.LED_status == LED_Dark) {
            LED2_OFF;
        } else if (LED_CTRL_STR.LED_status == LED_Bright) {
            if (LED_CTRL_STR.led_work_times >= ALWAYS_BRIGHT) {
                LED2_ON;
            } else {
                LED2_ON;
                rt_thread_mdelay(LED_CTRL_STR.led_work_times);
                LED2_OFF;
                rt_thread_mdelay(LED_CTRL_STR.led_work_times);
            }
        } else if (LED_CTRL_STR.LED_NUM == LED3) {
            if (LED_CTRL_STR.led_work_times >= ALWAYS_BRIGHT) {
                LED3_ON;
            } else {
                LED3_ON;
                rt_thread_mdelay(LED_CTRL_STR.led_work_times);
                LED3_OFF;
                rt_thread_mdelay(LED_CTRL_STR.led_work_times);
            }
        }
    }
}

/*------------------------------------------定义LED控制结构体----------------------------------------------*/
LED_CTRL led1_ctrl_struct = {
    .LED_NUM = LED1,
    .LED_status = LED_Bright,
    .led_work_times = 500,
    .p_led_ctrl = led_ctrl_function,
};


/*-----------------------------------------------定义线程栈--------------------------------------------------*/
static struct rt_thread led_thread;

/* 定义线程控栈时要求RT_ALIGN_SIZE个字节对齐 */
ALIGN(RT_ALIGN_SIZE)
/* 定义线程栈 */
static rt_uint8_t rt_led_thread_stack[256];


static void led_thread_entry(void* parameter) {
    while (1) {
        led1_ctrl_struct.p_led_ctrl(led1_ctrl_struct);
    }
    rt_thread_mdelay(10);
}

void led_thread_init()
{
    rt_thread_init(&led_thread,                             //线程控制块
                "led",                                  //线程名字
                led_thread_entry,                       //线程入口函数
                RT_NULL,                                //线程入口参数
                &rt_led_thread_stack[0],                //线程栈起始地址
                sizeof(rt_led_thread_stack),            //线程栈大小
                5,                                      //线程优先级
                20);                                    //线程时间片
    rt_thread_startup(&led_thread);                     //启动LED线程
}
INIT_APP_EXPORT(led_thread_init);

