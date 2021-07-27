#ifndef _CAN_H_
#define _CAN_H_
#include "stm32f10x.h"
#include "stdint.h"


#define CAN_RCV_BUF					        50
#define CAN_SEND_DATA_BUF					50

/**
 * 定义中断接收
 */
#define CAN1_RX_INT_ENABLE			1
#define CAN2_RX_INT_ENABLE			1

#define CAN1_PRPH                   1                       //定义CAN外设
#define CAN2_PRPH                   0                   

#define CAN1_BAUD					500						//500K波特率
#define CAN2_BAUD					500						//500K波特率

#define CAN1_INT					
#define CAN2_INT					

#define STD_FORMAT                  0
#define EXTD_FORMAT                 1

#define FIFOX0                      0
#define FIFOX1                      1

#define Locked                      1
#define NoLock                      0




/**
 * 定义CAN操作数据结构
 */
typedef struct  CAN_DATA_OPERATING
{
	uint32_t CANID;					//CANID
    uint8_t Data[8];				//
    uint8_t DataLen;      			//CAN
    uint8_t FrameType;    			//0--数据帧，1--远程帧
    uint8_t FrameFormat;  			//0--标准帧，1--扩展帧
    uint8_t Priority;   			//
    uint8_t Mark;         			//0--无效，1--有效
}CAN_DATA_OPERATING;


/**
 * 定义CAN接
 */
typedef struct CAN_RECV {
	CAN_DATA_OPERATING CAN_Recive_buff[CAN_RCV_BUF];
	uint16_t buf_lenth;
	uint8_t lock_flag;                              //
}CAN_RECV_BUF;
extern  CAN_RECV_BUF  CAN1_RECV_BUF;				//CAN1接收缓冲区
extern  CAN_RECV_BUF  CAN2_RECV_BUF;				//CAN2接收缓冲区




typedef struct CAN_SEND {
	CAN_DATA_OPERATING CAN_Send_buff[CAN_SEND_DATA_BUF];
	uint16_t buf_lenth;
	uint8_t lock_flag;                              //
}CAN_SEND_BUF;
extern  CAN_SEND_BUF  CAN1_SEND_BUF;				//CAN1发送缓冲，APP负责填充
extern  CAN_SEND_BUF  CAN2_SEND_BUF;				//CAN2发送缓冲，APP负责填充


void CAN_PRPH_Init(void);

CAN_RECV_BUF* can_receive(CAN_TypeDef* can);



#endif



