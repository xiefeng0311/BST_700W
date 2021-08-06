#ifndef _CHRG_CTRL_H_
#define _CHRG_CTRL_H_
#include "stdint.h"
#include "PWM.h"

#define CHRG_Channle1                       1
#define CHRG_Channle2                       2
#define START_CHRGING                       0
#define CHRGING                             1
#define WAITING                             0
#define EXECUTING                           1

#define Finished                    1
#define UnFinished                  0
#define AC_STAGE_NUM                6

typedef enum
{
	NO_Loader,
    BAT_Low_FAULT,
    BAT_High_FAULT,
    PFC_FAULT,
	PRE_CHRG,									
	CC_CHRG,									    
	CV_CHRG,
    CHRG_Finished,									
    CHRG_END,
}Chrg_ST_em;



extern uint16_t Target_I_PWM_CTRL;

typedef struct {
    uint16_t I_Collect_Times;
    uint16_t I_Period_Flags;
    uint16_t I_Avrg_Flags;
    uint16_t I1_Target_buf[300];
    uint16_t I2_Target_buf[300];   
}CHRG_I_CTRL_str;
extern CHRG_I_CTRL_str  I_chrg_ctrl_obj;

//充电器管理，主要用于调整中心电流值
typedef struct CHRG_CTRL {
    uint16_t Target_I_PWM_CTRL;
    uint16_t Target_I_Value;
    uint16_t Fully_V_Value;
    uint16_t I_Channel1;
    uint16_t I_Channel2;
    uint16_t V_BAT1;
    uint16_t V_BAT2;
    uint16_t CV_I_REF;
    uint8_t  FdBack_CTRL_Channel;
    uint8_t  CHRG_STT;
    uint8_t  IPP_Adjust;                    //电流纹波调节
    uint8_t  PRE_I_Adjust;                  //预充电流调节完成
    uint8_t  CC_I_Adjust;                   //横流调整完成标志
}CHRG_CTRL_STR;
extern CHRG_CTRL_STR  chrg_ctrl_obj;        //充电管理

#endif

