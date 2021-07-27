#include  "Chrg_Ctrl.h"
#include  "GPIO.h"
#include  "PWM.h"
#include  "MCU_ADC.h"
#include <stdint.h>
#include "rtthread.h"


/**
 * @brief 此文件用于管理充电器各项功能
 * @brief 通过多个线程进行管理 -- Thread1: 电流检测线程，Thread2: 电压检测线程，Thread3: 温度检测线程，Thead4:PFC检测线程
 */
/**
 * @brief define chrg_ctrl_st enum
 * 
 */


static uint8_t chargering_step = START_CHRGING;

static Chrg_ST_em Chrg_Status = NO_Loader;

typedef enum
{
	AC_Normal,
	AC_OVER,									
	AC_UnderV,									   
    AC_END,
}AC_st_em;
static AC_st_em PFC_Status = AC_Normal;

CHRG_I_CTRL_str  I_chrg_ctrl_obj = {
    .I_Collect_Times = 0,
    .I_Period_Flags = UnFinished,
    .I_Avrg_Flags = UnFinished,
    .I1_Target_buf = { 0 },
    .I2_Target_buf = {0},
};


CHRG_CTRL_STR  chrg_ctrl_obj = {
    .Target_I_PWM_CTRL = PWM_FM_200K,
    .Target_I_Value = 0,
    .Fully_V_Value = BAT_FULL_ADC,
    .I_Channel1 = 0,
    .I_Channel2 = 0,
    .V_BAT1 = 0,
    .V_BAT2 = 0,
    .FdBack_CTRL_Channel = 0,
    .CHRG_STT = NO_Loader,
    .IPP_Adjust = WAITING,
    .PRE_I_Adjust = UnFinished,
    .CC_I_Adjust = UnFinished,
};



/**
 * @brief I_Thread
 * @brief 电流线程主要用于采集平均电流值
 */
static struct rt_thread I_thread;
/* 定义线程控栈时要求RT_ALIGN_SIZE个字节对齐 */
ALIGN(RT_ALIGN_SIZE)
/* 定义线程栈 */
static rt_uint8_t rt_I_thread_stack[256];


static void I_thread_entry(void* parameter) {
    uint16_t I1_Value = 0;
    uint16_t I2_Value = 0;
    uint16_t i = 0;
    uint32_t sum1 = 0;
    uint32_t sum2 = 0;
    while (1) {
        //求一个周期内电流平均值
        if (I_chrg_ctrl_obj.I_Period_Flags == Finished) {
            for (i = 0; i < I_chrg_ctrl_obj.I_Collect_Times; i++) {
                sum1 += I_chrg_ctrl_obj.I1_Target_buf[i];
                sum2 += I_chrg_ctrl_obj.I2_Target_buf[i];
            }
            chrg_ctrl_obj.I_Channel1 = sum1 / I_chrg_ctrl_obj.I_Collect_Times;
            chrg_ctrl_obj.I_Channel2 = sum1 / I_chrg_ctrl_obj.I_Collect_Times;
            I_chrg_ctrl_obj.I_Collect_Times = 0;                    //清零计数值
            I_chrg_ctrl_obj.I_Avrg_Flags = Finished;
            I_chrg_ctrl_obj.I_Period_Flags = UnFinished;
        }
    }
}

void I_thread_init(void) {
    rt_thread_init(&I_thread,                                   //线程控制块
                "I_Monitor",                                    //线程名字
                I_thread_entry,                                 //线程入口函数
                RT_NULL,                                        //线程入口参数
                &rt_I_thread_stack[0],                          //线程栈起始地址
                sizeof(rt_I_thread_stack),                      //线程栈大小
                2,                                              //线程优先级
                20);  
    rt_thread_startup(&I_thread);                               //启动电流线程
}
INIT_APP_EXPORT(I_thread_init);                                 //初始化电流线程


/**
 * @brief 定义电压监测线程
 * @brief 此线程主要用于监测充电器各通道电压并根据此电压定义充电器的当前状态
 */
static struct rt_thread V_thread;
/* 定义线程控栈时要求RT_ALIGN_SIZE个字节对齐 */
ALIGN(RT_ALIGN_SIZE)
/* 定义栈空间大小 */
static rt_uint8_t rt_V_thread_stack[256];

static void V_thread_entry(void * parameter) 
{
    while(1) {
        chrg_ctrl_obj.V_BAT1 = AC_Singal_Parameter.BAT1;
        chrg_ctrl_obj.V_BAT2 = AC_Singal_Parameter.BAT2;
        if ((chrg_ctrl_obj.V_BAT1 <= BAT_LOADER_ADC) && (chrg_ctrl_obj.V_BAT2 <= BAT_LOADER_ADC)) {
            chrg_ctrl_obj.FdBack_CTRL_Channel = 0;
            chrg_ctrl_obj.CHRG_STT = NO_Loader;
        } else if (chrg_ctrl_obj.V_BAT1 >= chrg_ctrl_obj.V_BAT2) {
            chrg_ctrl_obj.FdBack_CTRL_Channel = CHRG_Channle2;
            //BAT2 not on line
            if (chrg_ctrl_obj.V_BAT2 <= BAT_LOADER_ADC) {
                chrg_ctrl_obj.FdBack_CTRL_Channel = CHRG_Channle1;
                if (chrg_ctrl_obj.V_BAT1 < BAT_PRECHR_ADC) {
                    chrg_ctrl_obj.CHRG_STT = BAT_Low_FAULT;
                } else if (chrg_ctrl_obj.V_BAT1 < BAT_CC_ADC) {
                    chrg_ctrl_obj.CHRG_STT = PRE_CHRG;
                } else if (chrg_ctrl_obj.V_BAT1 < BAT_FULL_ADC) {
                    chrg_ctrl_obj.CHRG_STT = CC_CHRG;
                } else if(chrg_ctrl_obj.V_BAT1  < BAT_PRT_ADC) {         //定义过压错误
                    chrg_ctrl_obj.CHRG_STT = CV_CHRG;
                } else {
                    chrg_ctrl_obj.CHRG_STT = BAT_High_FAULT;
                }
            } else if (chrg_ctrl_obj.V_BAT2 < BAT_PRECHR_ADC) {
                chrg_ctrl_obj.CHRG_STT = BAT_Low_FAULT;
            } else if(chrg_ctrl_obj.V_BAT2 < BAT_CC_ADC) {
                chrg_ctrl_obj.CHRG_STT = PRE_CHRG;
            } else if (chrg_ctrl_obj.V_BAT2 < BAT_FULL_ADC) {
                chrg_ctrl_obj.CHRG_STT = CC_CHRG;
            } else if (chrg_ctrl_obj.V_BAT2  < BAT_PRT_ADC) {
                chrg_ctrl_obj.CHRG_STT = CV_CHRG;
            } else {
                chrg_ctrl_obj.CHRG_STT = BAT_High_FAULT;
            }
        } else{
            chrg_ctrl_obj.FdBack_CTRL_Channel = CHRG_Channle1;
            //BAT1 not on line
            if (chrg_ctrl_obj.V_BAT1 <= BAT_LOADER_ADC) {
                chrg_ctrl_obj.FdBack_CTRL_Channel = CHRG_Channle2;
                if (chrg_ctrl_obj.V_BAT2 < BAT_PRECHR_ADC) {
                    chrg_ctrl_obj.CHRG_STT = BAT_Low_FAULT;
                } else if (chrg_ctrl_obj.V_BAT2 < BAT_CC_ADC) {
                    chrg_ctrl_obj.CHRG_STT = PRE_CHRG;
                } else if (chrg_ctrl_obj.V_BAT2 < BAT_FULL_ADC) {
                    chrg_ctrl_obj.CHRG_STT = CC_CHRG;
                } else if(chrg_ctrl_obj.V_BAT2  < BAT_PRT_ADC) {         //定义过压错误
                    chrg_ctrl_obj.CHRG_STT = CV_CHRG;
                } else {
                    chrg_ctrl_obj.CHRG_STT = BAT_High_FAULT;
                }
            } else if (chrg_ctrl_obj.V_BAT1 < BAT_PRECHR_ADC) {         //BAT1状态判断
                chrg_ctrl_obj.CHRG_STT = BAT_Low_FAULT;
            } else if(chrg_ctrl_obj.V_BAT1 < BAT_CC_ADC) {
                chrg_ctrl_obj.CHRG_STT = PRE_CHRG;
            } else if (chrg_ctrl_obj.V_BAT1 < BAT_FULL_ADC) {
                chrg_ctrl_obj.CHRG_STT = CC_CHRG;
            } else if (chrg_ctrl_obj.V_BAT1 < BAT_PRT_ADC) {
                chrg_ctrl_obj.CHRG_STT = CV_CHRG;
            } else {
                chrg_ctrl_obj.CHRG_STT = BAT_High_FAULT;
            }
        }
    }
}

void V_thread_init(void)
{
    rt_thread_init(&V_thread,                                   //线程控制块
                "V_Monitor",                                    //线程名字
                V_thread_entry,                                 //线程入口函数
                RT_NULL,                                        //线程入口参数
                &rt_V_thread_stack[0],                          //线程栈起始地址
                sizeof(rt_V_thread_stack),                      //线程栈大小
                3,                                              //线程优先级
                20); 
    rt_thread_startup(&V_thread);                               //启动电压控制线程
}
INIT_APP_EXPORT(V_thread_init);                                 //初始化电压线程

/**
 * @brief 定义充电器管理线程
 * @brief 此线程主要用于充电的充电管理
 */
static struct rt_thread CHRG_CTRL_thread;
/* 定义线程控栈时要求RT_ALIGN_SIZE个字节对齐 */
ALIGN(RT_ALIGN_SIZE)
/* 定义栈空间大小 */
static rt_uint8_t CHRG_CTRL_thread_stack[256];

static void chargering_start(void)
{
    //启动：PWM频率从200K逐步降到120K
    if (chrg_ctrl_obj.Target_I_PWM_CTRL < PWM_FM_120K) {
        chrg_ctrl_obj.Target_I_PWM_CTRL++;
        set_f(chrg_ctrl_obj.Target_I_PWM_CTRL);         
    } else {
        chargering_step = CHRGING;
    }
}

static void chrg_stop(void)
{
    power_on_disenabled();
    chrg_ctrl_obj.Target_I_PWM_CTRL = PWM_FM_200K;
    chrg_ctrl_obj.Target_I_Value = 0;
    chrg_ctrl_obj.Fully_V_Value = BAT_FULL_ADC;
    chrg_ctrl_obj.I_Channel1 = 0;
    chrg_ctrl_obj.I_Channel2 = 0;
    chrg_ctrl_obj.V_BAT1 = 0;
    chrg_ctrl_obj.V_BAT2 = 0;
    chrg_ctrl_obj.FdBack_CTRL_Channel = 0;
    chrg_ctrl_obj.CHRG_STT = NO_Loader;
    chrg_ctrl_obj.IPP_Adjust = WAITING;
    chrg_ctrl_obj.PRE_I_Adjust = UnFinished;
    chrg_ctrl_obj.CC_I_Adjust = UnFinished;
}

static void BAT_Fault(void)
{
    power_on_disenabled();
}

static void Pre_Chargering(void)
{
    chrg_ctrl_obj.Target_I_Value = PRE_I;
    if (I_chrg_ctrl_obj.I_Avrg_Flags == Finished) {                         //电流线程控制
        if (chrg_ctrl_obj.FdBack_CTRL_Channel == CHRG_Channle1) {
            if (chrg_ctrl_obj.PRE_I_Adjust == UnFinished) {
                if (chrg_ctrl_obj.I_Channel1 < chrg_ctrl_obj.Target_I_Value) {
                    chrg_ctrl_obj.Target_I_PWM_CTRL--;
                    set_f(chrg_ctrl_obj.Target_I_PWM_CTRL); 
                    chrg_ctrl_obj.IPP_Adjust = WAITING;              //暂停纹波调节
                    
                } else {
                    
                    chrg_ctrl_obj.IPP_Adjust = EXECUTING;           //开始执行纹波的调节
                    chrg_ctrl_obj.PRE_I_Adjust = Finished;          //不再调整中心电流
                }
            }
        } else if (chrg_ctrl_obj.FdBack_CTRL_Channel == CHRG_Channle2) {
            if (chrg_ctrl_obj.CC_I_Adjust == UnFinished) {
                if (chrg_ctrl_obj.I_Channel2 < chrg_ctrl_obj.Target_I_Value) {
                    chrg_ctrl_obj.Target_I_PWM_CTRL--;
                    set_f(chrg_ctrl_obj.Target_I_PWM_CTRL); 
                    chrg_ctrl_obj.IPP_Adjust = WAITING;
                } else {
                    
                    chrg_ctrl_obj.IPP_Adjust = EXECUTING;       //开始执行纹波的调节
                    chrg_ctrl_obj.PRE_I_Adjust = Finished;      //不再调节中心电流
                }
            }
        }
        I_chrg_ctrl_obj.I_Avrg_Flags = UnFinished;
    }
    
}

static void CC_Chargering(void) 
{
    chrg_ctrl_obj.Target_I_Value = CC_I;
    if (I_chrg_ctrl_obj.I_Avrg_Flags == Finished) {
        if (chrg_ctrl_obj.FdBack_CTRL_Channel == CHRG_Channle1) {
            if (chrg_ctrl_obj.CC_I_Adjust == UnFinished) {
                if (chrg_ctrl_obj.I_Channel1 < chrg_ctrl_obj.Target_I_Value) {
                    chrg_ctrl_obj.Target_I_PWM_CTRL--;
                    set_f(chrg_ctrl_obj.Target_I_PWM_CTRL); 
                    chrg_ctrl_obj.IPP_Adjust = WAITING;
                } else {
                    chrg_ctrl_obj.IPP_Adjust = EXECUTING;
                    chrg_ctrl_obj.CC_I_Adjust = Finished;
                }
            }
        } else if (chrg_ctrl_obj.FdBack_CTRL_Channel == CHRG_Channle2) {
            if (chrg_ctrl_obj.CC_I_Adjust == UnFinished) {
                if (chrg_ctrl_obj.I_Channel2 < chrg_ctrl_obj.Target_I_Value) {
                    chrg_ctrl_obj.Target_I_PWM_CTRL--;
                    set_f(chrg_ctrl_obj.Target_I_PWM_CTRL); 
                    chrg_ctrl_obj.IPP_Adjust = WAITING;
                } else {
                    chrg_ctrl_obj.IPP_Adjust = EXECUTING;
                    chrg_ctrl_obj.CC_I_Adjust = Finished;
                }
            }
        }
        I_chrg_ctrl_obj.I_Avrg_Flags = UnFinished;
    }
}

static void CV_Chargering(void) 
{
    if (chrg_ctrl_obj.FdBack_CTRL_Channel == CHRG_Channle1) {
        if (chrg_ctrl_obj.V_BAT1 <= chrg_ctrl_obj.Fully_V_Value) {
            chrg_ctrl_obj.Target_I_PWM_CTRL--;
            
        } else if (chrg_ctrl_obj.I_Channel1 < END_I) {
            chrg_ctrl_obj.CHRG_STT = CHRG_Finished;
        }
    } else if (chrg_ctrl_obj.FdBack_CTRL_Channel == CHRG_Channle2) {
        if (chrg_ctrl_obj.V_BAT2 <= chrg_ctrl_obj.Fully_V_Value) {
            chrg_ctrl_obj.Target_I_PWM_CTRL--;
        } else if (chrg_ctrl_obj.I_Channel2 < END_I) {
            chrg_ctrl_obj.CHRG_STT = CHRG_Finished;
        }
    }
}

static void Chargering_Finished(void) 
{
    power_on_disenabled();
    Relay1_OFF;
    Relay2_OFF;
}

static void Chargering_ctrl()
{
    
    if (chargering_step == START_CHRGING) {
        chargering_start();
    } else if (chargering_step == CHRGING) {
        if (chrg_ctrl_obj.CHRG_STT == PRE_CHRG) {
            Pre_Chargering();
        } else if (chrg_ctrl_obj.CHRG_STT == CC_CHRG) {
        }
    }
}


static void CHRG_CTRL_thread_entry(void * parameter) 
{
    while (1) {
        if (chrg_ctrl_obj.CHRG_STT == NO_Loader) {
            chrg_stop();
        } else if ((chrg_ctrl_obj.CHRG_STT == BAT_Low_FAULT) || (chrg_ctrl_obj.CHRG_STT == BAT_High_FAULT)) {
            BAT_Fault();
        } else  {
            Chargering_ctrl();
        } 
    }
}