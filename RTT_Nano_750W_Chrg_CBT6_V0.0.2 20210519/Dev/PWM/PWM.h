#ifndef __PWM_H__
#define __PWM_H__
#include "stdint.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "rtdef.h"


/* define the PWM adjust the source */
#define V1_PWM_src              TIM2
#define I1_PWM_src              TIM2
#define V2_PWM_src              TIM1
#define I2_PWM_src              TIM1
#define I1_2_PWM_src            TIM1

/**
 * @brief Define based on the voltage
 * @param the REF V of MCU is 3.3V, 12bit ADC
 * @param the preset value is 78V, proportionality is 1/40,so the adc of 78V is 2420;
 * @param the CV is 78.8V, so the adc of 78.8V is 2445
 */
#define BASE_V1   1000 //2420
#define BASE_V2   180 //2420
#define TG_V      2445    
/**
 * @brief Define based on the current
 * @param the REF V of MCU is 3.3V, 12bit ADC
 * @param the preset current is 0.5A, sampling resistor is 0.01r, so the V of 0.01 * 0.5 * 40 = 0.2V, the adc is 248
 * @param the Target I is 3.8A, 3.8 * 0.01 * 40 = 1.52V,so the adc of 1.52V is 1887
 */
#define BASE_I1         1000 //248
#define BASE_I2         180 //255248
#define TG_I            1887 
#define DRT             180
#define MAX_FM          360
#define MIN_FM          1200
/**
 * @brief 定义各个频率点的分频系数
 * @param 分频系数 = 72000000 / f 
 */
#define  PWM_FM_200K    360
#define  PWM_FM_120K    600
#define  PWM_FM_100K    720
#define  PWM_FM_80K     900
typedef enum {
    FM_200KHZ,
	FM_199KHZ,
    FM_198KHZ,
    FM_197KHZ,
    FM_196KHZ,
    FM_195KHZ,
    FM_194KHZ,
    FM_193KHZ,
    FM_192KHZ,
    FM_191KHZ,
    FM_190KHZ,
    FM_189KHZ,
    FM_188KHZ,
    FM_187KHZ,
    FM_186KHZ,
    FM_185KHZ,
    FM_184KHZ,
    FM_183KHZ,
    FM_182KHZ,
    FM_181KHZ,
    FM_180KHZ,
    FM_179KHZ,
    FM_178KHZ,
    FM_177KHZ,
    FM_176KHZ,
    FM_175KHZ,
    FM_174KHZ,
    FM_173KHZ,
    FM_172KHZ,
    FM_171KHZ,
    FM_170KHZ,
    FM_169KHZ,
    FM_168KHZ,
    FM_167KHZ,
    FM_166KHZ,
    FM_165KHZ,
    FM_164KHZ,
    FM_163KHZ,
    FM_162KHZ,
    FM_161KHZ,
    FM_160KHZ,
    FM_159KHZ,
    FM_158KHZ,
    FM_157KHZ,
    FM_156KHZ,
    FM_155KHZ,
    FM_154KHZ,
    FM_153KHZ,
    FM_152KHZ,
    FM_151KHZ,
    FM_150KHZ,
    FM_149KHZ,
    FM_148KHZ,
    FM_147KHZ,
    FM_146KHZ,
    FM_145KHZ,
    FM_144KHZ,
    FM_143KHZ,
    FM_142KHZ,
    FM_141KHZ,
    FM_140KHZ,
    FM_139KHZ,
    FM_138KHZ,
    FM_137KHZ,
    FM_136KHZ,
    FM_135KHZ,
    FM_134KHZ,
    FM_133KHZ,
    FM_132KHZ,
    FM_131KHZ,
    FM_130KHZ,
    FM_129KHZ,
    FM_128KHZ,
    FM_127KHZ,
    FM_126KHZ,
    FM_125KHZ,
    FM_124KHZ,
    FM_123KHZ,
    FM_122KHZ,
    FM_121KHZ,
    FM_120KHZ,
    FM_119KHZ,
    FM_118KHZ,
    FM_117KHZ,
    FM_116KHZ,
    FM_115KHZ,
    FM_114KHZ,
    FM_113KHZ,
    FM_112KHZ,
    FM_111KHZ,
    FM_110KHZ,
    FM_109KHZ,
    FM_108KHZ,
    FM_107KHZ,
    FM_106KHZ,
    FM_105KHZ,
    FM_104KHZ,
    FM_103KHZ,
    FM_102KHZ,
    FM_101KHZ,
    FM_100KHZ,
    FM_99KHZ,
    FM_98KHZ,
    FM_97KHZ,
    FM_96KHZ,
    FM_95KHZ,
    FM_94KHZ,
    FM_93KHZ,
    FM_92KHZ,
    FM_91KHZ,
    FM_90KHZ,
    FM_89KHZ,
    FM_88KHZ,
    FM_87KHZ,
    FM_86KHZ,
    FM_85KHZ,
    FM_84KHZ,
    FM_83KHZ,
    FM_82KHZ,
    FM_81KHZ,
    FM_80KHZ,
    FM_79KHZ,
    FM_78KHZ,
    FM_77KHZ,
    FM_76KHZ,
    FM_75KHZ,
    FM_74KHZ,
    FM_73KHZ,
    FM_72KHZ,
    FM_71KHZ,
    FM_70KHZ,
    FM_69KHZ,
    FM_68KHZ,
    FM_67KHZ,
    FM_66KHZ,
    FM_65KHZ,
    FM_64KHZ,
    FM_63KHZ,
    FM_62KHZ,
    FM_61KHZ,
    FM_60KHZ,
    FM_59KHZ,
    FM_58KHZ,
    FM_57KHZ,
    FM_56KHZ,
    FM_55KHZ,
    FM_54KHZ,
    FM_53KHZ,
    FM_52KHZ,
    FM_51KHZ,
    FM_50KHZ,
	FM_END,
}FM_NO;

extern uint16_t F_STEP_ARRAY[15];
extern FM_NO fm_pwm_no;
extern uint16_t FRQ_ARRAY[FM_END];
  
typedef struct {
	uint16_t pwm_fm_array[16];
}PWM_FM_STR;
extern PWM_FM_STR PWM_FM_struct;

typedef struct {
    uint16_t arr;
    uint16_t psc;
    uint16_t bs_V;
    uint16_t bs_I;
    void (*I_PWM_SET_Init)(uint16_t arr, uint16_t psc);
    void (*V_PWM_SET_Init)(uint16_t arr, uint16_t psc);
}VI_set_init;

/**
 * @brief 针对MCU控制方案目前采用PWM互补输出控制方式
 * @param the psc is the Prescaler of PWM
 * @param the arr is the period of PWM 
 * @param the D_ratio is the duty ratio of PWM 
 */
typedef struct {
    uint16_t arr;
    uint16_t psc;
    uint16_t D_ratio;
    void (*p_Complementation_PWM_Init)(uint16_t arr, uint16_t psc);
}Complementation_PWM_Init;


typedef enum {
    NO_Operation,
    INCREASE,           //up
    REDUCE,             //down
}set_drct;


extern uint16_t step;
extern uint16_t pwm_start_flag;

void PWM_PERPH_Init(void);

void PWM_start(TIM_TypeDef* TIMx);

void PWM_stop(TIM_TypeDef* TIMx);

void set_f(uint16_t f);

/* 调频 */

void FM_Adjust(uint16_t value);
void FM_auto_Adjust1(uint16_t v);

void FM_Up(uint16_t f_adjust_width);
void FM_Down(uint16_t f_adjust_width);

/* V1 reference set */
void V1_REF_SET(set_drct direction);
/* I1 reference set */
void I1_REF_SET(set_drct direction);
/* V2 reference set */
void V2_REF_SET(set_drct direction);
/* I2 reference set */
void I2_REF_SET(set_drct direction);
/* 自动调整到100KHz */
void FM_Down_To_100K(void);
/* 获取当前PWM设置参数 */
uint16_t get_PWM_Paramet(void);




#endif // !__PWM_H__


