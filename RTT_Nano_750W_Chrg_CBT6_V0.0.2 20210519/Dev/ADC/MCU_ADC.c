#include "MCU_ADC.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "Config.h"
#include "rtthread.h"
#include "rtdef.h"
#include "stddef.h"
#include "GPIO.h"
#include "misc.h"
#include "PWM.h"
#include "LED.h"
#include <stdint.h>
#include "Chrg_Ctrl.h"
#include "head.h"


/**
 * @brief 通过MCU的ADC采集外部数据并通过DMA进行传输；
 * @param 8个ADC通道 
 * @param 用一个二维数据进行滤波，并且采用先进先出的栈式滤波方式
 * @param 电流是比较特殊需要反应实时情况,不采用栈式滤波；并且目标电流通过完整一个周期内的平均值来活得
 *       （通过同步信号来获得周期）
 */



CHRG_ADC_DATA AC_Singal_Parameter;

uint8_t SYN_SIGNAL_Times;

uint8_t ADC_Finished_Flag;

CHRG_STEP charger_step = WAIT_MOD;                             //定义一个用于测试使用的步奏



CHRG_ERR_TYPE charger_err;       

CHRG_CHL charger_channel;

/* define CHRG ADC thread */
#define CHRG_THREAD_STACK_SIZE  256
#define CHRG_THREAD_PRIORITY    10
/* define CHRG PID Process thread */
#define CHRG_PID_STACK_SIZE     256
#define CHRG_PID_PRIORITY      10

enum {
    step0, 
    step1, 
    step2,
}pp_step_enum;

static uint8_t  pp_adjust_step = step0;

static uint16_t I_min = 0;
static uint16_t I_max = 0;
static uint16_t I_min_record = 0;
static uint16_t I_max_record = 0;
static uint16_t record_mark = 0;
static uint16_t pwm_step = 0;
static uint16_t avrg_bf_numb = 0;                           //滤波缓冲区数
static uint16_t finish_flag = 0;


static uint16_t PFC_IN_V[5] = {0, 1030, 1320, 1365, 4096};


static uint32_t ADC_buf_finish_flag = 0;


static uint16_t data_louth = 0;

static uint16_t data_count = 0;

uint16_t adc_finish_flag;

static uint16_t ref_vbat;

uint16_t ADCConvertedValue[NbrOfChannel] = {0};                             //DMA转换通道数组

uint16_t ADCConvertedBuf[NbrOfBuf][NbrOfChannel] = {0};                     //10组8通道缓冲，用于ADC滤波使用




static uint16_t AveragedBuf[NbrOfBuf] = {0};                                       //用于求平均buffer

//uint16_t ADCConvertedValue_buf[NbrOfChannel*300] = {0};




rt_timer_t ADCProcessSoftTimer;                         //软件定时器控制块指针,主要用于定时读取ADC

static CHRG_ADC_DATA CHRG_ADC_Value;

static CHRG_CHL CHRG_channel;



/**
 * @brief ADC的采集通过DMA中断接收接收处理采用邮件方式
 * @param 定义事件对象
 */
struct rt_mailbox mailbox_adc;
char mb_pool[40];





uint16_t get_frq(uint16_t* f_array, uint16_t f)
{
    uint16_t i;
    for (i = 0; i < FM_END; i++) {
        if (f <= FRQ_ARRAY[i]) {
            return i;
        }
    }
}

/**
 * @brief CHRG status process function
 * @param param
 */





uint16_t  get_avg_I(uint32_t present_I, uint16_t times)
{
    uint16_t avrg_I = 0;
    avrg_I = present_I / times;
    return avrg_I;
}
     
    


/* 测试使用 */
void test_full_chrg_process(uint16_t present_I, uint16_t present_V)
{
    if (present_V >= BAT_FULL_ADC) {
        if (present_I <= END_I) {
            //PWM_stop(I2_PWM_src);
        }
    }
}



/**
 * @brief ADC Pin Initialize
 * @param GPIO
 */
void ADC_Pin_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* Configure PA0,PA1,PA2,PA3,PA4,PA5,PA6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 
                                    | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}



/**
 * @brief ADC receive by DMA
 */
void ADC_DMA_Init()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
    /* ADC1 is on DMA1 channel1 configuration */
    DMA_DeInit(DMA1_Channel1);

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    DMA_InitTypeDef DMA_InitStructure = {
        .DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1->DR)),
        .DMA_MemoryBaseAddr = (uint32_t)ADCConvertedValue,
        .DMA_DIR = DMA_DIR_PeripheralSRC,
        .DMA_BufferSize = NbrOfChannel,
        .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
        .DMA_MemoryInc = DMA_MemoryInc_Enable,
        .DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord,
        .DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord,
        .DMA_Mode = DMA_Mode_Circular,
        .DMA_Priority = DMA_Priority_High,
        .DMA_M2M = DMA_M2M_Disable,
    };
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* 配置DMA中断源*/
    NVIC_InitTypeDef NVIC_InitStructure = { 
        .NVIC_IRQChannel = DMA1_Channel1_IRQn,
        .NVIC_IRQChannelPreemptionPriority = 1,
        .NVIC_IRQChannelSubPriority = 1,
        .NVIC_IRQChannelCmd = ENABLE,
    };
    NVIC_Init(&NVIC_InitStructure);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);  //配置DMA发送完成后产生中断
    
    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
}



/**
 * @brief ADC Mode Initialize
 */
void ADC_Mode_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
	/* ADC_TIM_BUS is on APB2 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); 	
    /* Enable ADC1clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /*---------------------- ADC1 configuration ----------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = NbrOfChannel;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 regular configuration */ 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_55Cycles5);


   /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);
   /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

   /* Enable ADC1 reset calibration register */   
    ADC_ResetCalibration(ADC1);
   /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
    /* Start ADC1 Software Conversion */ 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
 * @brief array assignment
 * 
 * @param p_org_array--original array
 * @param p_tg_array--target array
 * @param data_length--the length of the required operand
 */
void array_assg(uint16_t *p_org_array, uint16_t *p_tg_array, uint16_t chanl_num)
{
    uint16_t i;
    for (i = 0; i < chanl_num; i++) {
        p_tg_array[i] = p_org_array[i];
        //*(p_tg_array + i) = p_org_array[i];
    }
}


void ADCInit()
{
    rt_err_t result;
    ADC_Pin_Init();
    ADC_DMA_Init();
    ADC_Mode_Init();
    /* 初始化一个邮箱 */
    result = rt_mb_init(
        &mailbox_adc,
        "mb_adc",
        &mb_pool[0],
        sizeof(mb_pool) / 4,
        RT_IPC_FLAG_FIFO);
    if (result != RT_EOK) {
        rt_kprintf("init adc mailbox failed\n");
        return ;
    }
}

void array_ascend(uint16_t *p_tg_array, uint16_t data_length) 
{
    uint16_t data;
    uint16_t i;
    for (i = 0; i < data_length; i++) {
        if (p_tg_array[i] > p_tg_array[i+1]) {
            data = p_tg_array[i];
            p_tg_array[i] = p_tg_array[i+1];
            p_tg_array[i+1] = data;
        }
    }
}

/**
 * @brief array_swap: 此函数将DMA接收的缓冲数据转换成用于求平均数组
 * @param p_original原二维数组指针
 * @param p_target目标二维数组指针
 */
static void array_swap(uint16_t (*p_original)[NbrOfChannel], uint16_t (*p_target)[NbrOfBuf])
{
    uint16_t ch_num;
    uint16_t data_num;
    for (ch_num = 0; ch_num < NbrOfChannel; ch_num++) {
        for (data_num = 0; data_num < NbrOfBuf; data_num++) {
            p_target[ch_num][data_num] = p_original[data_num][ch_num];
        } 
    }
}


/**
 * @brief ascending_order用于将数组按从小到大排序
 * 
 * @param p_array 数据缓冲数组
 * @param data_num 数组长度
 */
void  ascending_order(uint16_t *p_array, uint16_t data_num) 
{
    uint16_t data;
    uint16_t i;
    for (i = 0; i < data_num; i++) {
        if (p_array[i] > p_array[i+1]) {
            data = p_array[i];
            p_array[i] = p_array[i+1];
            p_array[i+1] = data;
        }
    }
}

/**
 * @brief array_average: 采用平滑队列滤波方案
 * @param p_array 数据缓冲数组
 * @param chanel_num 数组元素号
 * @param data_num 数组数据个数
 * @return uint16_t 返回平均值结果
 */
uint16_t array_average(uint16_t *p_array, uint8_t chanel_num, uint16_t data_length) 
{
    static uint32_t data_sum = 0;
    static uint16_t i = 0;
    static uint16_t result = 0;
    uint16_t m = 0;
    
    if (data_length > sizeof(AveragedBuf)) {
        return SYS_ERR;
    } else {
        if (i < (data_length - 1)) {
            AveragedBuf[i] = p_array[chanel_num];
            i++;
        } else {
            AveragedBuf[++i] = p_array[chanel_num];
            for (m = 0; m < data_length; m++) {
                data_sum += AveragedBuf[m];
            }
            result =  (uint16_t)(data_sum / data_length);
            for (; i > 1; i--) {
                AveragedBuf[i-1] = AveragedBuf[i];
            }
            i = data_length - 1;
            data_sum = 0;
        }
    }
    return result;
}

/**
 * @brief 数据转移
 * 主要用于电流滤波
 */
void adc_data_carry (uint16_t *target_array, uint16_t *prim_array) {
    uint16_t i = 0;
    for (i = 0; i < ADC_CHNL_END; i++) {
        target_array[i] = prim_array[i];
    }
}

/**
 * @brief ADC_Average求出平均值
 * 
 * @param p_array -- 二维数组
 * @param channel_num -- 通道号
 * @param data_lenth -- 数据长度
 * @return uint16_t -- 返回值
 */
static uint16_t ADC_Average(uint16_t p_array[NbrOfBuf][NbrOfChannel], uint16_t channel_num, uint16_t data_lenth) 
{
    uint32_t data_sum = 0;
    uint16_t i = 0;
    uint16_t m = 0;
    uint16_t result = 0;
    if (data_lenth > NbrOfBuf) {
        return SYS_ERR;
    } else {
        for (i = 0; i < data_lenth; i++) {
            data_sum += p_array[i][channel_num];
        }
        result = data_sum / data_lenth;
    }
    return result;
}

/**
 * @brief ADC_filter 用于ADC滤波
 * 
 * @param p_array -- 用于滤波使用的二维数组
 * @param p_s_array -- DMA采用输入数组
 */
void ADC_filter(uint16_t p_array[NbrOfBuf][NbrOfChannel], uint16_t p_s_array[NbrOfChannel])
{
    static uint8_t full_flag = 0;
    static uint8_t data_times_record = 0;
    static uint8_t filter_times = 0;
    uint8_t i = 0;
    if (data_times_record < NbrOfBuf) {
        for (i = 0; i < NbrOfChannel; i++) {
            p_array[data_times_record][i] = p_s_array[i];
        }
        data_times_record++;
        if (data_times_record == NbrOfBuf) {
            for (i = 0; i < ADC_CHNL_END; i++) {
                AC_Singal_Parameter.ADC_data_array[i] = ADC_Average(p_array, i, NbrOfChannel);
            }
            ADC_Finished_Flag = ADC_FINISHED;
        }
    } else {
        //采用队列方式进行滤波
        if (filter_times < NbrOfBuf) {
            for (i = 0; i < NbrOfChannel; i++) {
                p_array[filter_times][i] = p_s_array[i];
            }
            for (i = 0; i < ADC_CHNL_END; i++) {
                AC_Singal_Parameter.ADC_data_array[i] = ADC_Average(p_array, i, NbrOfChannel);
            } 
            filter_times++;
            if (filter_times == NbrOfBuf) {
                filter_times = 0;
            }  
        } 
        ADC_Finished_Flag = ADC_FINISHED;
    }
    
}



int ADCProcessInit()
{
    ADCInit();
    // ADCProcessSoftTimer = rt_timer_create("ADCProcessSoftTimer", /* 软件定时器的名称 */
    //                     ADC_ProcessSoftTimer_callback,/* 软件定时器的回调函数 */
    //                     0,			/* 定时器超时函数的入口参数 */
    //                     10,   /* 10ms soft time,100Hz */
    //                     RT_TIMER_FLAG_PERIODIC );
    //                     /* 软件定时器HARD_TIMER模式 周期模式 */
	// if (ADCProcessSoftTimer != NULL) {
    //     rt_timer_start(ADCProcessSoftTimer);
    // }
	return 0;
}

INIT_APP_EXPORT(ADCProcessInit);


// ******此二维数组函数是根据数组数据在内存中是连续存储原理转换成一维数组的方式进行处理的******
/*
static uint16_t ADC_filter(uint16_t *p_array, uint16_t breadth, uint16_t data_length)
{
    uint32_t data_sum = 0;
    uint16_t i;

    for (i = 0; i < data_length; i++) {
        data_sum += *p_array;
        p_array += breadth;
    }

    return (uint16_t)(data_sum / data_length);
}

void CHRG_ADC_thread_entry(void *parameter)
{
    rt_kprintf("enter CHRG_ADC process\n\n");
    uint16_t data_breadth = 0;
    data_breadth = sizeof(ADCConvertedValue)/sizeof(uint16_t);               //求出一维数组的长度（字节数）
    while (1) {

        if (adc_finish_flag == 1) {
            CHRG_ADC_Value.Temperature = ADC_filter(&ADCConvertedBuf[0][0], data_breadth, NbrOfBuf);
            rt_kprintf("Temperature = %d\n\n", CHRG_ADC_Value.Temperature);
            CHRG_ADC_Value.BAT1 = ADC_filter(&ADCConvertedBuf[0][1], data_breadth, NbrOfBuf);
            rt_kprintf("BAT1 = %d\n\n", CHRG_ADC_Value.BAT1);
            CHRG_ADC_Value.BAT2 = ADC_filter(&ADCConvertedBuf[0][2], data_breadth, NbrOfBuf);
            rt_kprintf("BAT2 = %d\n\n", CHRG_ADC_Value.BAT2);
            CHRG_ADC_Value.V1 = ADC_filter(&ADCConvertedBuf[0][3], data_breadth, NbrOfBuf);
            rt_kprintf("V1 = %d\n\n", CHRG_ADC_Value.V1);
            CHRG_ADC_Value.I1 = ADC_filter(&ADCConvertedBuf[0][4], data_breadth, NbrOfBuf);
            rt_kprintf("I1 = %d\n\n", CHRG_ADC_Value.I1);
            CHRG_ADC_Value.V2 = ADC_filter(&ADCConvertedBuf[0][5], data_breadth, NbrOfBuf);
            rt_kprintf("V2 = %d\n\n", CHRG_ADC_Value.V2);
            CHRG_ADC_Value.I2 = ADC_filter(&ADCConvertedBuf[0][6], data_breadth, NbrOfBuf);
            rt_kprintf("I2 = %d\n\n", CHRG_ADC_Value.I2);
        }
        rt_thread_mdelay(200);
    }
}           
*/

/**
 * @brief 测试使用
 * 
 */
void cope_data_buf(void)
{
    uint16_t i = 0;
    if (ADC_buf_finish_flag == 0) {
        if (data_count < NbrDataOfBuf) {
            for (i = 0; i < NbrOfChannel; i++,data_count++) {
                //ADCConvertedValue_buf[data_count] = ADCConvertedValue[i];
            }
        } else {
            ADC_buf_finish_flag = 1;
        }
    } 
}



void ACV_IN_start(void)
{
    //PWM_start(I2_PWM_src);                            //直接启动
    /* 检测PFCV输入启动 */
    static uint16_t start_courent = 0;
    start_courent++;
    if (start_courent >= 30) {
        PWM_start(I2_PWM_src);
        
        start_courent = 0;
    }
    
}





void CHRG_ADC_thread_entry(void *parameter)
{
    static uint16_t ms_delay = 0;
    static uint16_t i = CHRG1I_CHNL4;
    rt_kprintf("enter CHRG_ADC process\n\n");
    
    while (1) {
        ms_delay++;
        
            if (ms_delay >= 300) {
                rt_kprintf("I1 = %d \n\n", ADCConvertedValue[CHRG1I_CHNL4]);
                // rt_kprintf("current_frq = %d \n\n", current_frq);
                // if (data_count >= NbrDataOfBuf) {
                //     for (i = CHRG1I_CHNL4; i < NbrDataOfBuf; i+=NbrOfChannel) {
                //         rt_kprintf("%d\n", ADCConvertedValue_buf[i]+ADCConvertedValue_buf[i+2]);
                //     }
                //     data_count = 0;
                //     ms_delay = 0;
                //     ADC_buf_finish_flag = 0;
                //     rt_kprintf("===========================\n");
                // }
            }
            
        
        
        //charger_stts_check();
        
        rt_thread_mdelay(10);
    }
}


void Chrg_adc_acquisition_thread(void)
{
    rt_thread_t tid;
    tid = rt_thread_create("CHRG_ADC_process",
                           CHRG_ADC_thread_entry, RT_NULL,
                           CHRG_THREAD_STACK_SIZE, CHRG_THREAD_PRIORITY, 10);
    if (tid != RT_NULL)  rt_thread_startup(tid);

    return ;
}
INIT_APP_EXPORT(Chrg_adc_acquisition_thread);




void CHRG_PID_thread_entry(void *parameter)
{
    rt_kprintf("enter CHRG_PID process\n\n");
    uint16_t data_breadth = 0;
    data_breadth = sizeof(ADCConvertedValue);               //求出一维数组的长度（字节数）
    while (1) {

        rt_thread_mdelay(200);
    }
}

void  CHRG_PID_process_thread(void)
{
    rt_thread_t tid;
    tid = rt_thread_create("CHRG_PID_process",
                           CHRG_PID_thread_entry, RT_NULL,
                           CHRG_PID_STACK_SIZE, CHRG_PID_PRIORITY, 10);
    if (tid != RT_NULL)  rt_thread_startup(tid);

    //return 0;
}

/**
 * @brief 电流纹波调整函数（只在目标电流已经调整好，才开始调整纹波电流）
 * @brief 纹波调整不关心具体那一路反馈控制，只根据当前状态调整到目标电流
 */
static uint8_t IPP_STT_RCRD[15] = {0};                              //纹波电流状态记录
static uint8_t IPP_Adjusted_Record[15] = {0};                       //用于标记是否已调节过
static uint16_t IPP_Adjust_Data_Record[15] = {0};                   //各点纹波调整值记录
static uint16_t IPP_Adjust_Data[15] = {0};
void current_pp_adjust(uint16_t target_I)
{
    static uint16_t data_times = 0;
    static uint16_t data_num = 0;
    static uint16_t i = 0;
    uint16_t Wave_I = 0;
    uint16_t n = 0;
    //uint16_t pwm_data = 0;
    Wave_I = target_I / 10;
    if (pp_adjust_step == step0) {
        data_times++;
        if (period_signl_recd >= 2) {
            data_num = data_times / Data_Border;                         //均分观测点    
            pp_adjust_step = step1;
            data_times = 0;
            period_signl_recd = 0;                              //重新开始
        }
    } else if (pp_adjust_step == step1) {
        if (period_signl_recd < 2) {
            data_times++;
            if (data_times >= ((i+1) * Data_Border)) {
                if (chrg_ctrl_obj.FdBack_CTRL_Channel == CHRG_Channle1) {
                    if (chrg_ctrl_obj.I_Channel1 < (target_I - Wave_I)) {
                        IPP_STT_RCRD[i] = LOWER;
                    } else if (chrg_ctrl_obj.I_Channel1 > (target_I + Wave_I)) {
                        IPP_STT_RCRD[i] = HIGHER;
                    } else {
                        IPP_STT_RCRD[i] = 0;
                    }
                } else if (chrg_ctrl_obj.FdBack_CTRL_Channel == CHRG_Channle2) {
                    if (chrg_ctrl_obj.I_Channel2 < (target_I - Wave_I)) {
                        IPP_STT_RCRD[i] = LOWER;
                    } else if (chrg_ctrl_obj.I_Channel2 > (target_I + Wave_I)) {
                        IPP_STT_RCRD[i] = HIGHER;
                    } else {
                        IPP_STT_RCRD[i] = 0;
                    }
                }
                i++;
                if (i == data_num) {
                    i = 0;
                    data_times = 0;
                    pp_adjust_step = step2;
                    period_signl_recd = 0;
                }
            }
        } else {
            i = 0;
            data_times = 0;
            pp_adjust_step = step2;
            period_signl_recd = 0;
        }
    } else if (pp_adjust_step == step2) {
        if (period_signl_recd < 2) {
            data_times++;
            n = data_times / Data_Border;
            if (IPP_Adjusted_Record[n] == UnFinished) {
                if (IPP_STT_RCRD[n] == LOWER) {
                    if (IPP_Adjust_Data_Record[n] == 0) {
                        IPP_Adjust_Data_Record[n] = get_PWM_Paramet();
                        IPP_Adjust_Data_Record[n]--;
                    } else {
                        IPP_Adjust_Data_Record[n]--;
                    }
                    set_f(IPP_Adjust_Data_Record[n]);
                } else if (IPP_STT_RCRD[n] == HIGHER) {
                    if (IPP_Adjust_Data_Record[n] == 0) {
                        IPP_Adjust_Data_Record[n] = get_PWM_Paramet();
                        IPP_Adjust_Data_Record[n]++;
                    } else {
                        IPP_Adjust_Data_Record[n]++;
                    }
                    set_f(IPP_Adjust_Data_Record[n]);
                } 
                IPP_Adjusted_Record[n] = Finished;
            } 
            if (data_times >= ((i+1) * Data_Border)) {
                if (chrg_ctrl_obj.FdBack_CTRL_Channel == CHRG_Channle1) {
                    if (chrg_ctrl_obj.I_Channel1 < (target_I - Wave_I)) {
                        IPP_STT_RCRD[i] = LOWER;
                    } else if (chrg_ctrl_obj.I_Channel1 > (target_I + Wave_I)) {
                        IPP_STT_RCRD[i] = HIGHER;
                    } else {
                        IPP_STT_RCRD[i] = 0;
                    }
                } else if (chrg_ctrl_obj.FdBack_CTRL_Channel == CHRG_Channle2) {
                    if (chrg_ctrl_obj.I_Channel2 < (target_I - Wave_I)) {
                        IPP_STT_RCRD[i] = LOWER;
                    } else if (chrg_ctrl_obj.I_Channel2 > (target_I + Wave_I)) {
                        IPP_STT_RCRD[i] = HIGHER;
                    } else {
                        IPP_STT_RCRD[i] = 0;
                    }
                }
                i++;
                if (i == data_num) {
                    i = 0;
                    data_times = 0;
                    period_signl_recd = 0;
                    for (n = 0; n < data_num; n++) {
                        IPP_STT_RCRD[n] = UnFinished;
                    }
                }
            }
        } else {
            i = 0;
            data_times = 0;
            period_signl_recd = 0;
            for (n = 0; n < data_num; n++) {
                IPP_STT_RCRD[n] = UnFinished;
            }
        }
    }
}


void DMA1_Channel1_IRQHandler(void)
{
    static uint32_t delay = 0;
    static uint32_t status_mb = 0;
    static uint16_t times = 0;
    uint16_t i = 0;
    delay++;
    if (DMA_GetITStatus(DMA1_IT_TC1)) {
        record_mark = 1;
        if (record_mark == 1) {
            if (pwm_step == 0) {
                CHRG_ADC_Value.PFC_Vin = array_average(ADCConvertedValue, PFCVin_CHNL7, NbrOfBuf);
                if (delay >= 20000) {
                    if (ACSingal_In() == 0) {
                        if ((PFC_STOPV < CHRG_ADC_Value.PFC_Vin) && (CHRG_ADC_Value.PFC_Vin < PFC_STV)) {
                            PWM_start(I1_2_PWM_src);
                            pwm_step = 1;
                        }
                    }
                }
            }else if (pwm_step == 1) {
                if (delay >= 32000) {
                    pwm_step = 2;
                    delay = 0;
                }
            } else if (pwm_step == 2) {
                adc_data_carry(AC_Singal_Parameter.ADC_data_array, ADCConvertedValue);
                ADC_filter(ADCConvertedBuf, ADCConvertedValue);
                if (chrg_ctrl_obj.CHRG_STT == PRE_CHRG) {
                    if  (chrg_ctrl_obj.PRE_I_Adjust == UnFinished) {            //调整中心电流值
                        if (period_signl_recd >= 1) {                           //同步信号首次出现
                            I_chrg_ctrl_obj.I1_Target_buf[I_chrg_ctrl_obj.I_Collect_Times] = AC_Singal_Parameter.I1;
                            I_chrg_ctrl_obj.I2_Target_buf[I_chrg_ctrl_obj.I_Collect_Times] = AC_Singal_Parameter.I2;
                            I_chrg_ctrl_obj.I_Collect_Times++;
                            if (period_signl_recd >= 2) {
                                I_chrg_ctrl_obj.I_Period_Flags = Finished;          //一个周期数据采集完成
                                period_signl_recd = 0;
                            }
                        }
                    } else if (chrg_ctrl_obj.PRE_I_Adjust == Finished) {        //开始调整纹波电流值
                        if (period_signl_recd >= 1) {                           //同步信号出现开始调整纹波电流
                            current_pp_adjust(PRE_I);
                        }
                    }
                    
                } else if (chrg_ctrl_obj.CHRG_STT == CC_CHRG) {
                    if (chrg_ctrl_obj.CC_I_Adjust == UnFinished) {
                        if (period_signl_recd >= 1) {
                            I_chrg_ctrl_obj.I1_Target_buf[I_chrg_ctrl_obj.I_Collect_Times] = AC_Singal_Parameter.I1;
                            I_chrg_ctrl_obj.I2_Target_buf[I_chrg_ctrl_obj.I_Collect_Times] = AC_Singal_Parameter.I2;
                            I_chrg_ctrl_obj.I_Collect_Times++;
                            if (period_signl_recd >= 2) {
                                I_chrg_ctrl_obj.I_Period_Flags = Finished;          //电流一个周期数据采集完成
                                period_signl_recd = 0;
                            }
                        }
                    } else if (chrg_ctrl_obj.CC_I_Adjust == Finished) {
                        if (period_signl_recd >= 1) {                           //同步信号出现开始调整纹波电流
                            current_pp_adjust(CC_I);
                        }
                    }
                }
            }
            record_mark = 0;
        }
    DMA_ClearITPendingBit(DMA1_IT_TC1); 
    }
}

