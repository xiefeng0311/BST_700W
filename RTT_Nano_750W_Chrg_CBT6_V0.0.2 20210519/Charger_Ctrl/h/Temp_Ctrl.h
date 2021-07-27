#ifndef _TEMP_CTRL_H_
#define _TEMP_CTRL_H_

#include "stdint.h"

typedef enum {
    T_Unlimited_Power,
    T_Limited_Power,
    T_Protected,
    T_end,
}T_Limit_Flag;

extern T_Limit_Flag T_limit_flag;


#endif /* _TEMP_CTRL_H_ */

