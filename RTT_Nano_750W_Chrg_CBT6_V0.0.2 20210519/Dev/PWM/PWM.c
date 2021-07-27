#include "PWM.h"
#include "rtthread.h"
#include "MCU_ADC.h"


extern struct rt_mailbox mailbox_adc;
FM_NO fm_pwm_no;

uint16_t pwm_start_flag = 0;
 

/**
 * @brief 预定义一个中心频率基础变化范围按照每峰峰值50个点
 */
uint16_t F_STEP_ARRAY[15] = {5, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

uint16_t FRQ_ARRAY[FM_END] = {360,362,364,365,367,369,371,373,375,377,379,381,383,385,387,389,391,393,396,398,400,402,404,407,409,411,
        414,416,419,421,424,426,429,431,434,436,439,442,444,447,450,453,456,459,462,465,468,471,474,477,480,483,486,490,493,497,500,503,
        507,511,514,518,522,526,529,533,537,541,545,550,554,558,563,567,571,576,581,585,590,595,600,605,610,615,621,626,632,637,643,649,
        655,661,667,673,679,686,692,699,706,713,720,727,735,742,750,758,766,774,783,791,800,809,818,828,837,847,857,867,878,889,900,911,
        923,935,947, 960,973,986,1000,1014,1029,1043,1059,1075,1091,1108,1125,1143,1161,1180,1200,1220,1241,1263,1286,1309,1333,1358,1385,
        1412,1440};

PWM_FM_STR PWM_FM_struct = {
	// .pwm_fm_array[FM_50KHZ] = FRQ_ARRAY[PWM_FM_50K],
	// .pwm_fm_array[FM_60KHZ] = FRQ_ARRAY[PWM_FM_60K],
	// .pwm_fm_array[FM_70KHZ] = FRQ_ARRAY[PWM_FM_70K],
	// .pwm_fm_array[FM_80KHZ] = FRQ_ARRAY[PWM_FM_80K],
	// .pwm_fm_array[FM_90KHZ] = FRQ_ARRAY[PWM_FM_90K],
	// .pwm_fm_array[FM_100KHZ] = FRQ_ARRAY[PWM_FM_100K],
	// .pwm_fm_array[FM_110KHZ] = FRQ_ARRAY[PWM_FM_110K],
	// .pwm_fm_array[FM_120KHZ] = FRQ_ARRAY[PWM_FM_120K],
	// .pwm_fm_array[FM_130KHZ] = FRQ_ARRAY[PWM_FM_130K],
	// .pwm_fm_array[FM_140KHZ] = FRQ_ARRAY[PWM_FM_140K],
	// .pwm_fm_array[FM_150KHZ] = FRQ_ARRAY[PWM_FM_150K],
	// .pwm_fm_array[FM_160KHZ] = FRQ_ARRAY[PWM_FM_160K],
	// .pwm_fm_array[FM_170KHZ] = FRQ_ARRAY[PWM_FM_170K],
	// .pwm_fm_array[FM_180KHZ] = FRQ_ARRAY[PWM_FM_180K],
	// .pwm_fm_array[FM_190KHZ] = FRQ_ARRAY[PWM_FM_190K],
	// .pwm_fm_array[FM_200KHZ] = FRQ_ARRAY[PWM_FM_200K],
};


#define PWM_THREAD_PRIORITY                 8
#define PWM_THREAD_TIMESLICE                10

uint16_t step = 0;


/**
 * @brief define initialize function
 * 
 * @param arr :period
 * @param psc :fre
 */
static void I1_PWM_SET_Init(uint16_t arr, uint16_t psc);
static void V1_PWM_SET_Init(uint16_t arr, uint16_t psc);
static void Complementation_PWM_SET_Init(uint16_t arr, uint16_t psc);
static void V2_PWM_SET_Init(uint16_t arr, uint16_t psc);
static void PWM_VI1_Init(void);
static void PWM_VI2_Init(void);


/**
 * @brief 创建一个PWM线程及线程栈
 */
ALIGN(RT_ALIGN_SIZE)
char pwm_thread_stack[1024];
struct rt_thread pwm_thread;

/**
 * @brief 创建一个PWM线程入口函数
 */
static void pwm_entry(void *parameter) 
{
    char *str;
    while (1) {
        
        if (rt_mb_recv(&mailbox_adc, (rt_uint32_t *)&str, RT_WAITING_FOREVER) == RT_EOK) {
            rt_kprintf("enter PWM control\n\n");
        }
        
        rt_thread_mdelay(1);
    }
    
}

/**
 * @brief define Complementary symmetric output initialize struct
 * @param TIM1是在APB2总线上，APB2总线时钟是72MHz不分频下，PWM频率72*10^6/ARR(KHz)
 */
static Complementation_PWM_Init Complementation_PWM_Init_str = {
    .arr = (PWM_FM_200K - 1),
    .psc = 1-1,
    .D_ratio = DRT,
    .p_Complementation_PWM_Init = Complementation_PWM_SET_Init
};

/**
 * @brief define initialize struct
 */
static VI_set_init channel1_str = { 
    .arr = (60000 - 1),                        //计数值4000，也就是TIM输入时钟的4000分频，目的是使得12位ADC都能得到调整
    .psc = 0,                                 //TIM输入时钟不分频
    .bs_V = BASE_V1,
    .bs_I = BASE_I1,
    .I_PWM_SET_Init = I1_PWM_SET_Init,
    .V_PWM_SET_Init = V1_PWM_SET_Init,
};
/**
 * @brief define initialize struct
 */
static VI_set_init channel2_str = { 
    .arr = (360 - 1),                        //计数值4000，也就是TIM输入时钟的4000分频，目的是使得12位ADC都能得到调整
    .psc = 1-1,
    .bs_V = BASE_V2,
    .bs_I = BASE_I2,
    .I_PWM_SET_Init = Complementation_PWM_SET_Init,
    .V_PWM_SET_Init = V2_PWM_SET_Init,
};

/**
 * @brief I&V benchmark set
 * @brief I1 set by TIME2_channel3，TIM2 bus frequency is 72MHz
 * @brief OUT by PB10
 * @param channel 
 */
void I1_PWM_SET_Init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseConfigStruct;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);                //enble TIME2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);               //enble PORTB
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);                   //Timer2部分重映射  TIM2_CH3->PB10

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                          //PB10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                     //AF_PP OUT
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                   //50MHz OUT
    GPIO_Init(GPIOB, &GPIO_InitStructure);                              //Init GPIOB

    /* Init time2 */
    TIM_TimeBaseConfigStruct.TIM_Period = arr;
    TIM_TimeBaseConfigStruct.TIM_Prescaler = psc;
    TIM_TimeBaseConfigStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseConfigStruct.TIM_CounterMode = TIM_CounterMode_Up;      
    TIM_TimeBaseInit(I1_PWM_src, &TIM_TimeBaseConfigStruct);                     //

    /* Init TIM2_CHL3 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC3Init(I1_PWM_src, &TIM_OCInitStructure);                             //the channel3 of TIM2
    
    TIM_OC3PreloadConfig(I1_PWM_src, TIM_OCPreload_Enable);   
    /* 因为想后续随时调整占空比所以不开启自动重载 */
	//TIM_ARRPreloadConfig(I1_PWM_src, ENABLE); 
    TIM_Cmd(I1_PWM_src, ENABLE);                      
}


/**
 * @brief I&V benchmark set
 * @brief V1 set by TIME2_channel4
 * @brief OUT by PB11
 * @param channel 
 */
void V1_PWM_SET_Init(uint16_t arr, uint16_t psc) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseConfigStruct;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);                //enble TIME2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);               //enble PORTB
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);                   //Timer2部分重映射  TIM2_CH4->PB11

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                          //PB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                     //AF_PP OUT
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                   //50MHz OUT
    GPIO_Init(GPIOB, &GPIO_InitStructure);                              //Init GPIOB

    /* Init time2 */
    TIM_TimeBaseConfigStruct.TIM_Period = arr;
    TIM_TimeBaseConfigStruct.TIM_Prescaler = psc;
    TIM_TimeBaseConfigStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseConfigStruct.TIM_CounterMode = TIM_CounterMode_Up;      
    TIM_TimeBaseInit(V1_PWM_src, &TIM_TimeBaseConfigStruct);                     //

    /* Init TIM2_CHL3 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC4Init(V1_PWM_src, &TIM_OCInitStructure);                             //the channel3 of TIM2
    /* 因为想后续随时调整占空比所以不开启自动重载 */
	//TIM_ARRPreloadConfig(V1_PWM_src, ENABLE); 
    TIM_OC4PreloadConfig(V1_PWM_src, TIM_OCPreload_Enable);   
    TIM_Cmd(V1_PWM_src, ENABLE); 
}



/**
 * @brief I&V Complementation PWM set
 * @brief set by TIME1_channel1
 * @brief OUT by PB13
 * @param channel 
 */
void Complementation_PWM_SET_Init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;
    TIM_BDTRInitTypeDef      TIM1_BDTRInitStruct;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);  //设置缺省值,这一步最好加上防止放到串口初始化后出问题
    TIM_OCStructInit(&TIM_OCInitStructure);          //设置缺省值,这一步最好加上防止放到串口初始化后出问题
    //TIM_ICStructInit(&TIM_ICInitStructure);        //设置缺省值,这一步最好加上防止放到串口初始化后出问题

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
    
//设置该引脚为复用输出功能,输出TIM1 CH1的PWM脉冲波形
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
//PA8与PB13采用互补输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    TIM_TimeBaseStructure.TIM_Period = arr; //自动重装载寄存器周期的值，溢出值
    TIM_TimeBaseStructure.TIM_Prescaler = psc; //时钟频率预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4; //设置时钟分割:输入捕获模式用来滤波
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    //TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//设置重复溢出次数，就是多少次溢出后进入中断，一般为0，只有高级定时器才有用
    TIM_TimeBaseInit(I2_PWM_src, &TIM_TimeBaseStructure);
    

    /* 配置死区时间 */
    TIM1_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Disable;
    TIM1_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Disable;
    TIM1_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM1_BDTRInitStruct.TIM_DeadTime = 0; //死区时间  72:1us 172:3us 205:5us
    TIM1_BDTRInitStruct.TIM_Break = TIM_Break_Disable;
    TIM1_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM1_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1,&TIM1_BDTRInitStruct);

    /* 因为2路电流设置采用的TIM1_CH1N，注意不是TIM1_CH1 */
    TIM_OCStructInit(&TIM_OCInitStructure);//设置缺省值,这一步最好加上
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
                                        
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; //输出极性:TIM输出比较极性高

    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(I2_PWM_src, &TIM_OCInitStructure);     
    TIM_OC1PreloadConfig(I2_PWM_src, TIM_OCPreload_Enable);

    //----------------------------------------------
	// /*TIM_OC2设置,作为触发信号，来复位TIM8*/
	// TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active; /*TIM1_OC2REF的模式为高电平有效*/
	// TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;  /*输出关闭*/
	// TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;/*输出关闭*/
	// TIM_OCInitStructure.TIM_Pulse = 50;                /*相位差*/
	// TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	// TIM_SelectOutputTrigger(TIM1,TIM_TRGOSource_OC2Ref);  /*TIM1_OC2REF作为主模式TRGO输出*/
	
	I1_2_PWM_src->BDTR |= 16;                               //
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    
//    TIM_CtrlPWMOutputs(I2_PWM_src,ENABLE);        //主输出使能
	
//    TIM_Cmd(I2_PWM_src, ENABLE);  //使能TIM1
}

void PWM_start(TIM_TypeDef* TIMx)
{
    TIM_CtrlPWMOutputs(I2_PWM_src, ENABLE);        //主输出使能
    TIM_Cmd(TIMx, ENABLE);  //使能TIM1
}

void PWM_stop(TIM_TypeDef* TIMx)
{
    TIM_CtrlPWMOutputs(TIMx, DISABLE);        //主输出使能
    TIM_Cmd(TIMx, DISABLE);  //使能TIM1
}


/**
 * @brief I&V benchmark set
 * @brief V2 set by TIME1_channel2
 * @brief OUT by PB14
 * @param channel 
 */
void V2_PWM_SET_Init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);  //设置缺省值,这一步最好加上防止放到串口初始化后出问题
    TIM_OCStructInit(&TIM_OCInitStructure);          //设置缺省值,这一步最好加上防止放到串口初始化后出问题
    //TIM_ICStructInit(&TIM_ICInitStructure);        //设置缺省值,这一步最好加上防止放到串口初始化后出问题

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  

    TIM_TimeBaseStructure.TIM_Period = arr; //自动重装载寄存器周期的值，溢出值
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //时钟频率预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:输入捕获模式用来滤波
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//设置重复溢出次数，就是多少次溢出后进入中断，一般为0，只有高级定时器才有用
    TIM_TimeBaseInit(V2_PWM_src, &TIM_TimeBaseStructure);
    
    
//设置该引脚为复用输出功能,输出TIM1 CH1的PWM脉冲波形
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* 因为2路电流设置采用的TIM1_CH2N，注意不是TIM1_CH2 */
    TIM_OCStructInit(&TIM_OCInitStructure);//设置缺省值,这一步最好加上
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low; //输出极性:TIM输出比较极性高

    TIM_OC2Init(V2_PWM_src, &TIM_OCInitStructure);           
    TIM_OC2PreloadConfig(V2_PWM_src, TIM_OCPreload_Enable);
    
    
    TIM_CtrlPWMOutputs(V2_PWM_src,ENABLE);        //主输出使能
    TIM_Cmd(V2_PWM_src, ENABLE);  //使能TIM1
}

/**
 * @brief Initialize first VI channel
 */
// void PWM_VI1_Init(void)
// {
//     channel1_str.I_PWM_SET_Init(channel1_str.arr, channel1_str.psc);
//     TIM_SetCompare3(I1_PWM_src, channel1_str.bs_I);
//     channel1_str.V_PWM_SET_Init(channel1_str.arr, channel1_str.psc);
//     TIM_SetCompare4(V1_PWM_src, channel1_str.bs_V);
// }

/**
 * @brief Initialize second VI channel
 */
// void PWM_VI2_Init(void)
// {
//     uint16_t tim1_bdtr = 0;
//     channel2_str.I_PWM_SET_Init(channel2_str.arr, channel2_str.psc);
//     TIM_SetCompare1(I2_PWM_src,channel2_str.bs_I);//通道1
//     tim1_bdtr = I2_PWM_src->BDTR;
    
//     channel2_str.V_PWM_SET_Init(channel2_str.arr, channel2_str.psc);
//     TIM_SetCompare2(V2_PWM_src, channel2_str.bs_V);//
//     rt_kprintf("tim1_bdtr = %d\n\n", tim1_bdtr);
// }


/**
 * @brief Initialize second VI channel
 */
void PWM_Control_Init(void)
{
    uint16_t tim1_bdtr = 0;
    Complementation_PWM_Init_str.p_Complementation_PWM_Init(Complementation_PWM_Init_str.arr, Complementation_PWM_Init_str.psc);
    TIM_SetCompare1(I1_2_PWM_src,Complementation_PWM_Init_str.D_ratio);//通道1
    tim1_bdtr = I2_PWM_src->BDTR;
    rt_kprintf("tim1_bdtr = %d\n\n", tim1_bdtr);
}


/**
 * @brief PWM peripheral init
 * @param the PWM of I1,I2,V1,V2 Init
 */
void PWM_PERPH_Init(void)
{
    // PWM_VI1_Init();
    // PWM_VI2_Init();
    PWM_Control_Init();
}

/* External automatic initialization */
INIT_DEVICE_EXPORT(PWM_PERPH_Init);

/**
 * @brief set the V1 reference V
 * 
 * @param direction : is INCREASE or REDUCE
 */
void V1_REF_SET(set_drct direction)
{
    static uint16_t duty_cycle = 0;
    if (duty_cycle == 0) {                  //首次初始化
        duty_cycle = channel1_str.bs_V;
    }
    if (direction == INCREASE) {
        duty_cycle++;
        TIM_SetCompare2(V1_PWM_src, duty_cycle);
    } else if (direction == REDUCE) {
        duty_cycle--;
        TIM_SetCompare2(V1_PWM_src, duty_cycle);
    } 
}


/**
 * @brief set the I1 reference V
 * 
 * @param direction : is INCREASE or REDUCE
 */
void I1_REF_SET(set_drct direction)
{
    static uint16_t duty_cycle = 0;
    if (duty_cycle == 0) {
        duty_cycle = channel1_str.bs_I;
    }
    if (direction == INCREASE) {
        duty_cycle++;
        TIM_SetCompare2(I1_PWM_src, duty_cycle);
    } else if (direction == REDUCE) {
        duty_cycle--;
        TIM_SetCompare2(I1_PWM_src, duty_cycle);
    }
}


/**
 * @brief set the V2 reference V
 * 
 * @param direction : is INCREASE or REDUCE
 */
void V2_REF_SET(set_drct direction)
{
    static uint16_t duty_cycle = 0;
    if (duty_cycle == 0) {
        duty_cycle = channel2_str.bs_V;
    }
    if (direction == INCREASE) {
        duty_cycle++;
        TIM_SetCompare2(V2_PWM_src, duty_cycle);
    } else if (direction == REDUCE) {
        duty_cycle--;
        TIM_SetCompare2(V2_PWM_src, duty_cycle);
    }
}


/**
 * @brief set the V2 reference V
 * 
 * @param direction : is INCREASE or REDUCE
 */
void I2_REF_SET(set_drct direction)
{
    static uint16_t duty_cycle = 0;
    if (duty_cycle == 0) {
        duty_cycle = channel2_str.bs_I;
    }
    if (direction == INCREASE) {
        duty_cycle++;
        TIM_SetCompare2(I2_PWM_src, duty_cycle);
    } else if (direction == REDUCE) {
        duty_cycle--;
        TIM_SetCompare2(I2_PWM_src, duty_cycle);
    }
}

/**
 * @brief init PWM thread
 * @param direction : is INCREASE or REDUCE
 */
void pwm_thread_init(void) 
{
    rt_thread_init(&pwm_thread,
                    "pwm_thred",
                    pwm_entry,
                    RT_NULL,
                    &pwm_thread_stack[0],
                    sizeof(pwm_thread_stack),
                    PWM_THREAD_PRIORITY,
                    PWM_THREAD_TIMESLICE);
    rt_thread_startup(&pwm_thread);
}
INIT_APP_EXPORT(pwm_thread_init);


uint16_t get_PWM_Paramet(void)
{
    uint16_t PWM_Paramet = 0;
    PWM_Paramet = I1_2_PWM_src->ARR;
    return PWM_Paramet;
}

void set_f(uint16_t f)
{
    TIM_SetAutoreload(I1_2_PWM_src, f);
    Complementation_PWM_Init_str.D_ratio = f / 2;
    TIM_SetCompare1(I1_2_PWM_src,Complementation_PWM_Init_str.D_ratio);
}


/**
 * @brief FM_Up：此函数用于抬高PWM的频率
 * @brief 抬高频率对应减小分频系数值 
 * @param f_adjust_width 调节PWM频率步长
 */
void FM_Up(uint16_t f_adjust_width)
{
    uint16_t Period;
    Period = I1_2_PWM_src->ARR;
    if (f_adjust_width == 0) {
        Period -= 1;
        TIM_SetAutoreload(I1_2_PWM_src, Period);
        Complementation_PWM_Init_str.D_ratio = Period / 2;
        TIM_SetCompare1(I1_2_PWM_src,Complementation_PWM_Init_str.D_ratio);
        // rt_kprintf("Period = %d\n\n", Period);
        // rt_kprintf("D_ratio = %d\n\n", Complementation_PWM_Init_str.D_ratio);
    } else {
        Period -= f_adjust_width;
        TIM_SetAutoreload(I1_2_PWM_src, Period);
        Complementation_PWM_Init_str.D_ratio = Period / 2;
        TIM_SetCompare1(I1_2_PWM_src,Complementation_PWM_Init_str.D_ratio);
        // rt_kprintf("Period = %d\n\n", Period);
        // rt_kprintf("D_ratio = %d\n\n", Complementation_PWM_Init_str.D_ratio);
    }
}

/**
 * @brief FM_Down：此函数用于降低PWM频率
 * @brief 降低频率对应增加分频系数
 * @param f_adjust_width 调节PWM频率步长
 */
void FM_Down(uint16_t f_adjust_width)
{
    uint16_t Period;
    Period = I1_2_PWM_src->ARR;
    if (f_adjust_width == 0) {
        Period += 1; 
        if (Period <= FRQ_ARRAY[FM_60KHZ]) {
            TIM_SetAutoreload(I1_2_PWM_src, Period);
            Complementation_PWM_Init_str.D_ratio = Period / 2;
            TIM_SetCompare1(I1_2_PWM_src,Complementation_PWM_Init_str.D_ratio);
            // rt_kprintf("Period = %d\n\n", Period);
            // rt_kprintf("D_ratio = %d\n\n", Complementation_PWM_Init_str.D_ratio);
        }
        
    } else {
        Period += f_adjust_width; 
        TIM_SetAutoreload(I1_2_PWM_src, Period);
        Complementation_PWM_Init_str.D_ratio = Period / 2;
        TIM_SetCompare1(I1_2_PWM_src,Complementation_PWM_Init_str.D_ratio);
        // rt_kprintf("Period = %d\n\n", Period);
        // rt_kprintf("D_ratio = %d\n\n", Complementation_PWM_Init_str.D_ratio);
    }
    
}

/**
 * @brief FM_Down：此函数用于降低PWM频率
 * @brief 降低频率对应增加分频系数
 * @param f_adjust_width 调节PWM频率步长
 */
void FM_Down_To_100K(void)
{
	uint16_t Period;
	Period = I1_2_PWM_src->ARR;
	if (Period <= FRQ_ARRAY[FM_100KHZ]) {
		Period++;
		TIM_SetAutoreload(I1_2_PWM_src, Period);
    Complementation_PWM_Init_str.D_ratio = Period / 2;
    TIM_SetCompare1(I1_2_PWM_src,Complementation_PWM_Init_str.D_ratio);
	}
}


void FM_Adjust(uint16_t value)
{
    static uint16_t record;
    uint16_t Period;
    uint16_t v;
    if (value <= 620) {                                                     //对应0.5V输入
        Period = I1_2_PWM_src->ARR;
        if ((Period + 2) <= FRQ_ARRAY[FM_50KHZ]) {
            Period += 2;
            Complementation_PWM_Init_str.D_ratio = Period / 2;
            TIM_SetAutoreload(I1_2_PWM_src, Period);
            TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
        } else {
            Period = FRQ_ARRAY[FM_50KHZ];
            Complementation_PWM_Init_str.D_ratio = Period / 2;
            TIM_SetAutoreload(I1_2_PWM_src, Period);
            TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
        }
        
    } else if (value >= 3100) {
        Period = I1_2_PWM_src->ARR;
        if ((Period - 2) >= FRQ_ARRAY[FM_200KHZ]) {
            Period -= 2;
            Complementation_PWM_Init_str.D_ratio = Period / 2;
            TIM_SetAutoreload(I1_2_PWM_src, Period);
            TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
        } else {
            Period = FRQ_ARRAY[FM_200KHZ];
            Complementation_PWM_Init_str.D_ratio = Period / 2;
            TIM_SetAutoreload(I1_2_PWM_src, Period);
            TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
        }
    } else {
        v = (uint16_t)(value - 620)/16;
        v *= 7;
        record = FRQ_ARRAY[FM_50KHZ] - v;
        Period = I1_2_PWM_src->ARR;
        if (record > FRQ_ARRAY[FM_50KHZ]) {
            if ((Period - 2) < record) {
                Period -= 2;
                Complementation_PWM_Init_str.D_ratio = Period / 2;
                TIM_SetAutoreload(I1_2_PWM_src, Period);
                TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
            } else {
                Period = record;
                Complementation_PWM_Init_str.D_ratio = Period / 2;
                TIM_SetAutoreload(I1_2_PWM_src, Period);
                TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
            }
            
        }
    }
}

void FM_auto_Adjust() 
{
    uint16_t Period;
    Period = I1_2_PWM_src->ARR;
    if (Period <= FRQ_ARRAY[FM_120KHZ]) {
        if ((Period + 2) < FRQ_ARRAY[FM_120KHZ]) {
            Period += 2;
            Complementation_PWM_Init_str.D_ratio = Period / 2;
            TIM_SetAutoreload(I1_2_PWM_src, Period);
            TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
        } else {
            Period = FRQ_ARRAY[FM_100KHZ];
            Complementation_PWM_Init_str.D_ratio = Period / 2;
            TIM_SetAutoreload(I1_2_PWM_src, Period);
            TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
        }
    } else if (Period >= FRQ_ARRAY[FM_100KHZ]) {
        if ((Period - 2) > FRQ_ARRAY[FM_120KHZ]) {
            Period -= 2;
            Complementation_PWM_Init_str.D_ratio = Period / 2;
            TIM_SetAutoreload(I1_2_PWM_src, Period);
            TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
        } else {
            Period = FRQ_ARRAY[FM_120KHZ];
            Complementation_PWM_Init_str.D_ratio = Period / 2;
            TIM_SetAutoreload(I1_2_PWM_src, Period);
            TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
        }
    } else {
        Period = FRQ_ARRAY[FM_120KHZ];
        Complementation_PWM_Init_str.D_ratio = Period / 2;
        TIM_SetAutoreload(I1_2_PWM_src, Period);
        TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
    }
}


uint16_t FM_Jump(uint16_t num)
{
	// uint16_t Period;
	// static uint16_t step = 0;
	// if (step == 0) {
    //     Period = I1_2_PWM_src->ARR;
    //     if (Period <= (PWM_FM_struct.pwm_fm_array[num - 1])) { 
    //         Period += 4;
    //         Complementation_PWM_Init_str.D_ratio = Period / 2;
    //         TIM_SetAutoreload(I1_2_PWM_src, Period);
    //         TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
    //     } else step = 1;
	// } else if (step == 1) {
    //     Period = I1_2_PWM_src->ARR;
    //     if (Period >= PWM_FM_struct.pwm_fm_array[num + 1]) {
    //         Period -= 4;
    //         Complementation_PWM_Init_str.D_ratio = Period / 2;
    //         TIM_SetAutoreload(I1_2_PWM_src, Period);
    //         TIM_SetCompare1(I1_2_PWM_src, Complementation_PWM_Init_str.D_ratio);
    //     } else step = 0;
	// }
}


uint16_t FM_select_num(uint16_t Period, PWM_FM_STR pwm_str)
{
	uint16_t i = FM_END;
	while (--i) {
		if (Period < pwm_str.pwm_fm_array[i]) {
			return i;
		}
	}
}


void FM_auto_Adjust1(uint16_t v) 
{
	// static uint16_t step_pwm = 0;
	// static uint16_t fm;
	// static uint16_t delay;
    // uint16_t Period;
	// uint16_t num;
    // uint16_t adc_v = 0;
    // if (v <= ADC_0V5) {
    //     FM_Jump(FM_60KHZ);
        
    // } else if (v >= ADC_3V2) {
    //     FM_Jump(FM_190KHZ);
        
    // } else {
	// 	if (v > ADC_0V5) {
	// 		adc_v = (v - ADC_0V5) / 4;              //ADC对应得PWM计数值
	// 		fm = PWM_FM_50K - adc_v;                //目标PWM脉冲值
    //         num = FM_select_num(fm, PWM_FM_struct);
	// 		FM_Jump(num);
	// 	}
    // }
}

void dead_up(void)
{
    static uint16_t dead_t = 0;
    TIM_BDTRInitTypeDef      TIM1_BDTRInitStruct;
    dead_t = 16;
    if (dead_t < 0xff) {
        I1_2_PWM_src->BDTR |= dead_t;
    } else {
        dead_t = 0;
    }
    
}
void dead_down(void)
{
    // static uint16_t dead_t;
    // dead_t = I1_2_PWM_src->BDTR;
    // dead_t &= 0x00FF;
    // TIM_BDTRInitTypeDef      TIM1_BDTRInitStruct;
    // if (dead_t > 0) {
    //     dead_t--;
    //     I1_2_PWM_src->BDTR += dead_t;
    // }
}


