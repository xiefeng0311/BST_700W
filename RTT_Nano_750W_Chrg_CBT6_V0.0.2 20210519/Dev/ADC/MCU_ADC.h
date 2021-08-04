#ifndef __MCU_ADC_H__
#define __MCU_ADC_H__
//#include "core_cm3.h"
#include "stdint.h"
#include "PWM.h"

/**
 * @brief define ADC_channel 
 */
#define  T_CHNL_ADC                 ADC_Channel_0               //PA0
#define  BAT1_CHNL_ADC              ADC_Channel_1               //PA1
#define  BAT2_CHNL_ADC              ADC_Channel_2               //PA2
#define  CHGV1_CHNL_ADC             ADC_Channel_3               //PA3
#define  CHGI1_CHNL_ADC             ADC_Channel_4               //PA4
#define  CHGV2_CHNL_ADC             ADC_Channel_5               //PA5
#define  CHGI2_CHNL_ADC             ADC_Channel_6               //PA6

/**
 * @brief define ADC_EVENT
 */
#define ADC_EVENT                   1                           //定义ADC事件
#define ADC_FINISHED                1                           //定义ADC完成，包括滤波完成

#define LOWER                       1                           //比目标低
#define HIGHER                      2                           //比目标高

#define Data_Border                 20                          //用于数据观察分界单位

/**
 * @brief I Direction
 */
#define I_Rise                   1                           //定义电流上升方向
#define I_Drop                   2                           //定义电流下降方向

#define I_PP_FRST                1                           //定义第一个峰峰值
#define I_PP_SCND                2                           //定义第二个峰峰值

#define CLLECT_NUM               6                           //定义一个周期中采用6个点




/**
 * @brief I Direction   terminus
 */
#define ORIGIN                   1                           //定义周期的起点
#define TERMINUS                 2                           //定义周期的重点

/**
 * @brief define ADC channel quantity
 */
#define NbrOfChannel                8    
#define NbrOfBuf                    10  
//test
#define NbrDataOfBuf                800*3 

#define DataFilter                  100
/**
 * @brief define bat ON Value
 * @param voltage gain is 1/40,BAT ONline is 10V,ADC_IN = 0.25V; ADC = 4096 / 3.3 * 0.25
 */
#define BAT_LOADER_ADC              310
/**
 * @brief define bat PRE_CHRG Model
 * @param voltage gain is 1/40,BAT PRECHRG is 47.5V,ADC_IN = 47.5V/40; ADC = 4096*1.1875 / 3.3
 */
#define BAT_PRECHR_ADC              1474
/**
 * @brief define bat CC Model
 * @param voltage gain is 1/40,BAT CC_CHRG start value is 57V,ADC_IN = 57V/40; ADC = 4096*1.425 / 3.3
 */
#define BAT_CC_ADC                  1769
/**
 * @brief define FULL Model
 * @param voltage gain is 1/40,BAT Full_CHRG value is 78.85V,ADC_IN = 78.85V/40; ADC = 4096*1.97125 / 3.3
 */
#define BAT_FULL_ADC                2447
/**
 * @brief define PRT Model
 * @param voltage gain is 1/40,BAT Full_CHRG value is 80V,ADC_IN = 82V/40; ADC = 4096*2.05 / 3.3
 */
#define BAT_PRT_ADC                2482

/**
 * @brief BATV differ
 * @param gain is 1/40,BAT Full_CHRG value is 78.85V,ADC_IN = 1V/40; ADC = 31
 */
#define V_Diff                31

/**
 * @brief PFCVin Start V
 * @param PFCVin < 1.064V start LLC, 
 */
#define PFC_STV                     1321

/**
 * @brief  Stop LLC V
 * @param PFCVin < 0.83V stop LLC,
 */
#define PFC_STOPV                   1030

/**
 * @brief  Limit LLC P
 * @param PFCVin < 1.1 limited LLC output power,
 */
#define PFC_LPV                   1365


/**
 * @brief define High Tempreture Protect(5K) V=5*4096/15
 * @param voltage 
 */
#define HTMP_PRT                    1365
/**
 * @brief define Low Tempreture Protect(5K) V=20*4096/30
 * @param voltage 
 */
#define LTMP_PRT                    2731

/**
 * @brief define pre_charger current
 * @param current gain is 40,R=0.01,Current=1A
 */
#define PRE_I                      496  
#define PRE_I2                     992


/**
 * @brief define protect current
 * @param current gain is 40,R=0.01,Current=5A
 */
#define PRTT_I                       1240


/**
 * @brief define cc current
 * @param current gain is 40,R=0.01,Current=4A
 */
#define CC_I                       1986

/**
 * @brief define cc current
 * @param current gain is 40,R=0.01,Current=2A
 */
#define I_2A                       992

/**
 * @brief define cc current
 * @param current gain is 40,R=0.01,Current=1A
 */
#define I_1A                       496

/**
 * @brief 实际测试电流纹波是>800mA, 800多mA是波峰和波谷之间值，ADC是0.1ms采集一次，PFC
 * @param current gain is 40,R=0.01,Current=0.016A
 */
#define OFFSET_I1                   10
/**
 * @brief define END current
 * @param current gain is 40,R=0.01,Current=0.032A
 */
#define OFFSET_I2                   16

/**
 * @brief define END current
 * @param current gain is 40,R=0.01,Current=0.048A
 */
#define OFFSET_I3                   24


/**
 * @brief difference  current
 * @param current gain is 40,R=0.01,Current=0.048A
 */
#define DIF_I                     32


/**
 * @brief define END current
 * @param current gain is 40,R=0.01,Current=0.1A
 */
#define START_I                   50

/**
 * @brief define END current
 * @param current gain is 40,R=0.01,Current=0.5A
 */
#define END_I                       248




/**
 * @brief test 测试使用，定义ADC最小值及最大值
 *
 */
#define ADC_DIF         4
#define ADC_0V5         4
#define ADC_3V2         4010





/**
 * @brief define chrg status
 */


/**
 * @brief define chrg status
 */
typedef enum {
    WAIT_MOD,
    PREPROCESS,
    CC_MOD,
    CV_MOD,
    STEP_END,
}CHRG_STEP;
extern CHRG_STEP charger_step; 

/**
 * @brief define chrg err type
 */
typedef enum {
    NO_ERR,
    BAT_ERR,
    TEMP_ERR,
    CC_ERR,
    ERR_END,
}CHRG_ERR_TYPE;
extern CHRG_ERR_TYPE charger_err;                    


/**
 * @brief Channel NO
 */
typedef enum {
    TMEP_CHNL0,
    /* channel1 */
    BAT1_CHNL1,
    BAT2_CHNL2,
    CHRG1V_CHNL3,
    CHRG1I_CHNL4,
    CHRG2V_CHNL5,
    CHRG2I_CHNL6,
    PFCVin_CHNL7,
    ADC_CHNL_END,
}ADC_CHNL;


/**
 * @brief define CHRG_ADC process data struct
 * @param V1,I1,V2,I2
 * @param rf=3.3V,12bit ADC,resolution ratio:0.00081V/bit
 */
typedef union {
    uint16_t Temperature;
    uint16_t BAT1;
    uint16_t BAT2;
    uint16_t V1;
    uint16_t I1;
    uint16_t V2;
    uint16_t I2;
    uint16_t PFC_Vin;                       //PFC以1/71比例输入ADC
    uint16_t ADC_data_array[ADC_CHNL_END];
}CHRG_ADC_DATA;
extern CHRG_ADC_DATA AC_Singal_Parameter;

/**
 * @brief define chrg channel
 */
typedef enum {
    NO,
    /* channel1 */
    CHNL1,
    CHNL2,
    CHNL_END,
}CHRG_CHL;
extern CHRG_CHL charger_channel;

/**
 * @brief enum AC input voltage
 */
typedef enum {
    PFC_NO_IN,
    /* channel1 */
    PFC_UnderV,
    PFC_START,
    PFC_OverV,
    PFC_END
}PFC_IN_ENUM;


extern char mb_pool[40];

extern uint8_t ADC_Finished_Flag;

extern struct rt_mailbox mailbox_adc;

extern uint16_t ADCConvertedValue[NbrOfChannel];

extern uint16_t ADCConvertedBuf[NbrOfBuf][NbrOfChannel];

extern uint16_t adc_finish_flag;

extern uint16_t pwm_step;


#endif


