#include "GPIO.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "rtdef.h"
#include "rtthread.h"
#include "stdio.h"
#include "PWM.h"
#include "Chrg_Ctrl.h"
#include "delay.h"
#include "drv_time.h"





uint16_t period_signl_recd = 0; 
//需要增加一个Fault信号监测



/**
 * @brief define thread parameter
 * 
 */
#define BUTTON_THREAD_STACK_SIZE    256
#define BUTTON_THREAD_PRIORITY      5

// 按键定义
static Button_t s_tBtnKey;	

/**
 * message queue structure
 */
static rt_mq_t button_mq = RT_NULL;//按键事件队列定义

/* Key按键按下时的电平，=0,按下时为低电平;=1,按下时为高电平 */
#define KeyPressedLevel 0

static uint8_t IsKeyDown(void) 		{if (KeyIn != KeyPressedLevel) return 0; return 1;}



void mcu_gpio_init(void)
{
    GPIO_InitTypeDef  PB_OUT_InitStructure = { 
        /* PB3-5 用于LED控制，PB6-9用于继电器控制MCU版目前只用PB8&9，PB14用于风扇控制，PB15用于认证使用 */
        .GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 
                    | GPIO_Pin_8 | GPIO_Pin_9| GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15,               
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_50MHz,
    };
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    //使能PB端口时钟
    GPIO_Init(GPIOB, &PB_OUT_InitStructure);					    		//根据设定参数初始化
    GPIO_SetBits(GPIOB,GPIO_Pin_5);						        				//PB.5 输出高
	GPIO_SetBits(GPIOB,GPIO_Pin_12);						        			//PB.12 输出高
    
    /* PA15用于14V输出控制 */
    GPIO_InitTypeDef PA_OUT_InitStructure = {
        .GPIO_Pin = GPIO_Pin_15,
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_50MHz,
    };
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	        //使能PB端口时钟
    GPIO_Init(GPIOA, &PA_OUT_InitStructure);					    //根据设定参数初始化

    /* PB2 用作输入，用于对AC的同步信号*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    //使能PB端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	        //中断时受复用寄存器控制的
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);    //中断源line2
    GPIO_InitTypeDef PB_IN_InitStructure = {
        .GPIO_Pin = GPIO_Pin_2,
        .GPIO_Mode = GPIO_Mode_IPU,                     //浮空输入
    };
    GPIO_Init(GPIOB, &PB_IN_InitStructure);					    //根据设定参数初始化
    EXTI_InitTypeDef EXTI_InitStructure = {

        .EXTI_Line = EXTI_Line2,                               //PB2采用输入中断
        .EXTI_Mode = EXTI_Mode_Interrupt,	
  	    .EXTI_Trigger = EXTI_Trigger_Rising,
  	    .EXTI_LineCmd = ENABLE,
    };
    EXTI_Init(&EXTI_InitStructure);
    NVIC_InitTypeDef NVIC_InitStructure = {
        .NVIC_IRQChannel = EXTI2_IRQn,                      //
        .NVIC_IRQChannelPreemptionPriority = 1,
        .NVIC_IRQChannelSubPriority = 2,
        .NVIC_IRQChannelCmd = ENABLE,
    };
    NVIC_Init(&NVIC_InitStructure); 
}




/**
 * @brief KEY is contrled by PB3
 * @brief PB3 need to configured to pull up input
 */
void key_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	/* PORTB is on APB2 bus */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);               //enable PORTB clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Key;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_Init(GPIO_Port_Key, &GPIO_InitStructure);                      //Inition GPIOB
}




void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	/* PORTB is on APB2 bus */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);               //enable PORTB clock
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_POWER_ON;                    //PB2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_Init(GPIO_Port_POWER_ON, &GPIO_InitStructure);                 //Inition GPIOB
}



static void ButtonVarInit(void)
{
	s_tBtnKey.IsKeyDownFunc=IsKeyDown;							    // 检测按键按下函数
	s_tBtnKey.wFilterCount=0;										// 滤波器计数器 
	s_tBtnKey.wFilterTime =BUTTON_FILTER_TIME;						// 滤波时间 
	s_tBtnKey.wLongCount =0;										// 长按计数器 
	s_tBtnKey.wLongTime=BUTTON_LONG_TIME;							// 长按时间 	
	s_tBtnKey.byState=0;											// 按键当前状态（按下还是弹起）
	s_tBtnKey.byKeyCodeUp=KEY_UP;									// 按键弹起的键值代码, 0表示不检测按键弹起 
	s_tBtnKey.byKeyCodeDown=KEY_DOWN;								// 按键按下的键值代码, 0表示不检测按键按下
	s_tBtnKey.byKeyCodeLong=KEY_LONG;								// 按键长按的键值代码, 0表示不检测长按 
}



/*******************************************************************************************************
** 函数: ButtonDetect,Button按键检测函数
**------------------------------------------------------------------------------------------------------
** 参数: ptBtn 按键结构体指针
** 返回: void
********************************************************************************************************/
static void ButtonDetect(Button_t *ptBtn)
{
	if(ptBtn->IsKeyDownFunc && ptBtn->IsKeyDownFunc()) 				// 检测按键函数是否存在，按键是否按下
	{

		if(ptBtn->wFilterCount < ptBtn->wFilterTime)				// 滤波检测，滤波计数器到达滤波时间
		{
			ptBtn->wFilterCount++;									// 计数器加一
			return;													// 退出检测函数
		}
		
		if(ptBtn->byState ==0 ) 									// 检测是否是按键按下							
		{
			ptBtn->byState = 1;
			rt_mq_send(button_mq,										// 写入（发送）队列的ID(句柄)
								&(ptBtn->byKeyCodeDown),  				// 写入（发送）的数据所对应地址 
								sizeof(ptBtn->byKeyCodeDown)			// 数据的长度 
								);
			return;
		}
		if( ptBtn->wLongCount++ == ptBtn->wLongTime) 				    // 检测是否是按键长按，长按计数器是否到达长按时间
		{
			rt_mq_send(button_mq,										// 写入（发送）队列的ID(句柄)
								&(ptBtn->byKeyCodeLong),  				// 写入（发送）的数据所对应地址 
								sizeof(ptBtn->byKeyCodeLong)			// 数据的长度 
								);
			return;
		}			
	}
	else 
	{		
		if(ptBtn->wFilterCount) 									    // 滤波检测，滤波计数器是否为0
		{
			ptBtn->wFilterCount--;									    // 计数器减一
			return;													    // 退出检测函数
		}			
		
		if(ptBtn->byState ==1 )										    // 检测是否是按键弹起
		{
			ptBtn->byState =0; 										
			rt_mq_send(button_mq,										// 写入（发送）队列的ID(句柄)
								&(ptBtn->byKeyCodeUp),  				// 写入（发送）的数据所对应地址 
								sizeof(ptBtn->byKeyCodeUp)			    // 数据的长度 
								);
		}
		
		ptBtn->wLongCount = 0;										    // 按键长按计数器清零
	}
	
}


void Channel_ctrl(channel_em channel, relay_ctrl_em ctrl_mdl)
{
    switch(channel) {
        case Channel1:
        switch(ctrl_mdl) {
            case Closed:
            case Opened:
            default : break;
        }
        case Channel2:
        switch (ctrl_mdl) {
            case Closed:
            case Opened:
            default : break;
        }
    }
}


void ButtonProj(void)
{
	//该函数在定时器中断回调函数中调用，定时中断周期为1ms
	ButtonDetect(&s_tBtnKey);										// 检测 K1 键 
	/****************************************************************************************/
	//用户添加自定义按键变量初始化
    //例如：ButtonDetect(&s_tBtnKeyN);	
	
	/****************************************************************************************/
}


void IO_PCI_Init(void)
{
	mcu_gpio_init();
    gpio_init();
	key_init();												// Button 按键GPIO初始化
    power_on_disenabled();
	ButtonVarInit();												// Button 按键配置初始化
}
INIT_DEVICE_EXPORT(IO_PCI_Init);


/*******************************************************************************************************
** 函数: button_thread_entry,获取按键事件并进行处理
**------------------------------------------------------------------------------------------------------
** 参数: void
** 返回: void
********************************************************************************************************/


void button_thread_entry(void *parameter)//用户消息处理入口函数
{
	rt_err_t uwRet = RT_EOK;
	uint8_t r_queue;//用于接收msg_mq消息队列信息
	
	button_mq = rt_mq_create("button_mq",							//消息队列名字
														1,  									//消息的最大长度, bytes
														256,										//消息队列的最大容量(个数)
														RT_IPC_FLAG_FIFO			//队列模式 FIFO
														);
	if(button_mq != RT_NULL)
		rt_kprintf("button_mq create success\n\n");
	
	//ButtonInit();//按键硬件接口初始化
	
	while(1)
	{  //获取队列信息
		uwRet = rt_mq_recv(button_mq,
												&r_queue,
												sizeof(r_queue),
												RT_WAITING_FOREVER
												);
		if(RT_EOK == uwRet )
		{
			switch(r_queue)//根据接收到的消息内容分别进行处理
			{
				case KEY_DOWN:
                    rt_kprintf("Receive message:KEY1(PB.3) Down\n\n");
                    
                break;
				case KEY_UP:
                    rt_kprintf("Receive message:KEY1(PB.3) Up\n\n");
                break;
				case KEY_LONG:
                    rt_kprintf("Receive message:KEY1(PB.3) LongPressed Down\n\n");
                break;
				default: 
                    rt_kprintf("No button Message!\n\n");
                break;
			}
		}
		else
		{
			rt_kprintf("数据接收错误，错误代码：0x%lx\n\n",uwRet);
		}
	}	
	
}
int button_process_init(void)
{
    rt_thread_t tid;


    tid = rt_thread_create("button_process",
                           button_thread_entry, RT_NULL,
                           BUTTON_THREAD_STACK_SIZE, BUTTON_THREAD_PRIORITY, 10);


    if (tid != NULL)
        rt_thread_startup(tid);
    return 0;
}
INIT_APP_EXPORT(button_process_init);


void power_on_enabled(void)
{
    RESET_FAULT;
    GPIO_WriteBit(GPIO_Port_POWER_ON, GPIO_Pin_POWER_ON, Bit_RESET);
}
/* 关闭电源的输出 */
void power_on_disenabled(void)
{
    FAULT_Protect;
    GPIO_WriteBit(GPIO_Port_POWER_ON, GPIO_Pin_POWER_ON, Bit_SET);
}





/**
 * @brief 中断线2用于同步信号的监测
 * 用于滤波和5ms超时保护
 */
void  EXTI2_IRQHandler(void)
{
	//    static rt_uint16_t step = 0;
	uint16_t times;
	times++;
	if (ACSingal_In() == 1) {
		period_signl_recd++;                    //ADC监测同步信号使用
		times = 0;
		Time3_Start();                          //启动5ms的监测		
    }
		
    // if (times > 20) {
    //     if (ACSingal_In() == 1) {
    //         period_signl_recd++;                    //ADC监测同步信号使用
    //         times = 0;
    //         Time3_Start();                          //启动5ms的监测
    //     }
	// 	EXTI_ClearITPendingBit(EXTI_Line2);  //清除LINE3上的中断标志位  
    // }
    // //5ms的保护采用定时器中断实现，并且在终端中启动定时器
	EXTI_ClearITPendingBit(EXTI_Line2);  //清除LINE3上的中断标志位  
}
