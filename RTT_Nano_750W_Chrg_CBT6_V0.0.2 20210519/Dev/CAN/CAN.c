#include "CAN.h"
//#include "SYS_Tick.h"
#include "stm32f10x_can.h"
#include "rtdef.h"
#include "rtthread.h"

CAN_RECV_BUF  CAN1_RECV_BUF;				//CAN1接收缓冲区
CAN_RECV_BUF  CAN2_RECV_BUF;				//CAN2接收缓冲区

CAN_SEND_BUF  CAN1_SEND_BUF;				//CAN1发送缓冲，APP负责填充
CAN_SEND_BUF  CAN2_SEND_BUF;				//CAN2发送缓冲，APP负责填充





static uint8_t CAN_Tx_Msg(CAN_TypeDef* can, CAN_DATA_OPERATING* can_send_data)
{
    uint8_t mbox;	  
	if(can->TSR & (1<<26))       mbox = 0;			//邮箱0为空
	else if(can->TSR & (1<<27))  mbox = 1;			//邮箱1为空
	else if(can->TSR & (1<<28))  mbox = 2;			//邮箱2为空
    else return 0XFF;								//无空邮箱,无法发送
    can->sTxMailBox[mbox].TIR = 0;					//清发送邮箱标识符寄存器
    /* 标准帧 */
    if(can_send_data->FrameFormat == 0) {
        can_send_data->CANID &= 0x7ff;					//取低11位stdid
		can_send_data->CANID <<= 21;
    } else {                                            //扩展帧
		can_send_data->CANID &= 0X1FFFFFFF;			    //取低32位extid
		can_send_data->CANID <<= 3;									   
	}	
    can->sTxMailBox[mbox].TIR |= can_send_data->CANID;		 
	can->sTxMailBox[mbox].TIR |= can_send_data->FrameFormat << 2;	  
	can->sTxMailBox[mbox].TIR |= can_send_data->FrameType << 1;
	can_send_data->DataLen &= 0X0F;					//得到低四位
	can->sTxMailBox[mbox].TDTR &= ~(0X0000000F);
	can->sTxMailBox[mbox].TDTR |= can_send_data->DataLen;		   //设置DLC.	

    //待发送数据存入邮箱.
	can->sTxMailBox[mbox].TDHR = (((uint32_t)can_send_data->Data[7]<<24)|
								  ((uint32_t)can_send_data->Data[6]<<16)|
 								  ((uint32_t)can_send_data->Data[5]<<8)|
								  ((uint32_t)can_send_data->Data[4]));
	can->sTxMailBox[mbox].TDLR = (((uint32_t)can_send_data->Data[3]<<24)|
								  ((uint32_t)can_send_data->Data[2]<<16)|
 								  ((uint32_t)can_send_data->Data[1]<<8)|
								  ((uint32_t)can_send_data->Data[0]));
	can->sTxMailBox[mbox].TIR |= 1<<0; //请求发送邮箱数据
	return mbox;	
}

/*
 *获得发送状态.
 *mbox:邮箱编号;
 *返回值:发送状态. 0,挂起;0X05,发送失败;0X07,发送成功.
 */
static uint8_t  CAN_Tx_Staus(CAN_TypeDef* can, uint8_t mbox)
{	
	uint8_t sta = 0;					    
	switch (mbox)
	{
		case 0: 
			sta |= can->TSR  & (1<<0);					//RQCP0
			sta |= can->TSR  & (1<<1);					//TXOK0
			sta |= ((can->TSR & (1<<26))>>24);			//TME0
			break;
		case 1: 
			sta |= can->TSR  & (1<<8)>>8;				//RQCP1
			sta |= can->TSR  & (1<<9)>>8;				//TXOK1
			sta |=((can->TSR & (1<<27))>>25);			//TME1	   
			break;
		case 2: 
			sta |= can->TSR  & (1<<16)>>16;			//RQCP2
			sta |= can->TSR  & (1<<17)>>16;			//TXOK2
			sta |=((can->TSR & (1<<28))>>26);			//TME2
			break;
		default:
			sta=0X05;									//邮箱号不对,肯定失败.
		break;
	}
	return sta;
} 


void CAN_Send(CAN_TypeDef* can, CAN_DATA_OPERATING* can_send_data)
{
    uint8_t mbox;
	uint16_t i = 0;	  	 			
	
    mbox = CAN_Tx_Msg(can, can_send_data);
	while((CAN_Tx_Staus(can, mbox)!=0X07) && (i<0x005))  i++;		//等待发送结束 0XFFF
	if(i >= 0x005)  return ;									//发送失败?
	return ;
}





/*
 *接收数据
 *fifox:邮箱号
 *id:标准ID(11位)/扩展ID(11位+18位)	    
 *de:0,标准帧;1,扩展帧
 *rtr:0,数据帧;1,远程帧
 *len:接收到的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
 *dat:数据缓存区
 * 因为采用中断接收因此只使用邮箱0就可以了
 */
void   CAN_Rx_Msg(CAN_TypeDef* can, uint8_t fifox, CAN_RECV_BUF * P_CAN_BUF)
{
    /* 获取CAN的标识符 */	
    P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].FrameFormat = (can->sFIFOMailBox[fifox].RIR & 0x04) >> 2;    
    
    if (P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].FrameFormat == STD_FORMAT) {
        P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].CANID = can->sFIFOMailBox[fifox].RIR >> 21;
    } else {
        P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].CANID = can->sFIFOMailBox[fifox].RIR >> 3;
    }
    P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].FrameType = can->sFIFOMailBox[fifox].RIR & 0x02;	//得到远程发送请求值.
    P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].DataLen = can->sFIFOMailBox[fifox].RDTR & 0x0F;	//得到DLC  
	P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].Data[0] = can->sFIFOMailBox[fifox].RDLR & 0XFF;
    P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].Data[1] = (can->sFIFOMailBox[fifox].RDLR>>8) & 0XFF;
    P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].Data[2] = (can->sFIFOMailBox[fifox].RDLR>>16) & 0XFF;
    P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].Data[3] = (can->sFIFOMailBox[fifox].RDLR>>24) & 0XFF;
    P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].Data[4] =  can->sFIFOMailBox[fifox].RDHR & 0XFF;
    P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].Data[5] = (can->sFIFOMailBox[fifox].RDHR>>8) & 0XFF;
    P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].Data[6] = (can->sFIFOMailBox[fifox].RDHR>>16) & 0XFF;
    P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].Data[7] = (can->sFIFOMailBox[fifox].RDHR>>24) & 0XFF; 
    P_CAN_BUF->buf_lenth++;                 //缓冲区数量加1

    if(fifox==0)       can->RF0R |= 0X20;//释放FIFO0邮箱
	else if(fifox==1)  can->RF1R |= 0X20;//释放FIFO1邮箱	 
}


CAN_RECV_BUF* can_receive(CAN_TypeDef* can)
{
    if (can == CAN1) {
        CAN_Rx_Msg(CAN1, FIFOX0, &CAN1_RECV_BUF);
        return &CAN1_RECV_BUF;
    } else if (can == CAN2) {
        CAN_Rx_Msg(CAN2, FIFOX0, &CAN1_RECV_BUF);
        return &CAN2_RECV_BUF;
    }
}


/*
确认是否进入了CAN发送中断
*/
uint8_t CAN_Confirm_In(CAN_TypeDef* can)
{
//	uint8_t i=0;
	if(can->TSR&0x00010101) return 1;
	else return 0;
}


/*
 *得到在FIFO0/FIFO1中接收到的报文个数.
 *fifox:0/1.FIFO编号;
 *返回值:FIFO0/FIFO1中的报文个数.
 */
uint8_t  CAN_Msg_Pend(CAN_TypeDef* can, uint8_t fifox)
{
	if(fifox == 0)       return can->RF0R & 0x03; 
	else if(fifox == 1)  return can->RF1R & 0x03; 
	else return  0;
}





/*
 *can口接收数据查询
 *buf:数据缓存区;	 
 *返回值:0,无数据被收到;
 *其他,接收的数据长度;
 */
uint8_t  CAN_Receive_Msg(CAN_TypeDef* can, CAN_RECV_BUF * P_CAN_BUF)
{		   		   
	
//	uint8_t  len; 
//	if(CAN_Msg_Pend(can, 0) == 0)  return 0;		//没有接收到数据,直接退出 	 
//  	CAN_Rx_Msg(can, 0, P_CAN_BUF); 	//读取数据
//	if (P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].CANID != 0x12 || 
//        P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].FrameFormat != 0 || 
//        P_CAN_BUF->CAN_Recive_buff[P_CAN_BUF->buf_lenth].FrameType != 0) len = 0;
//	return len;	
}






static void CAN_FNCT_Init(CAN_TypeDef* can, uint16_t can_baud)
{
    uint16_t  i = 0;
	
	
	can->MCR = 0X0000;
	can->MCR |= 1 << 0;            //请求初始化
	while ((can->MSR & (1 << 0)) == 0)
	{
		i++;
		if (i > 100)   return ;
	}
	can->MCR |= 0 << 7;
	can->MCR |= 1 << 6;
	can->MCR |= 0 << 5;             //睡眠模式由软件唤醒
	can->MCR &= ~( 1 << 4 );
	can->MCR |= 0 << 3;             //下个报文会覆盖原有报文
	can->MCR |= 0 << 2;             //优先级由标识符决定
	can->BTR = 0X00000000;
	if (can_baud == 500)
	{
		//can在APB1系统总线上，因此最高时钟频率是36MHz,波特率=1/(正常位时间)，
		//正常位时间=1*tq+tq*(TS1[3:0]+1)+tq*(TS2[2:0]+1),tq=(BRP[9:0]+1)*tpclk,CAN波特率预设计成500K,
		//baud=500K,tq=BRP*(1/36000000);
		//建议时间份额为16，时间段1为11，时间段2为4，跳转宽度为3
		can->BTR |= 0 << 30;
		can->BTR |= 3 << 24;             //重新同步跳转宽为(3+1)tq
		can->BTR |= 11 << 16;            //时间段1为(11+1)tq
		can->BTR |= 4 << 20;             //时间段2为(4+1)tq
		can->BTR |= 3 << 0;              //设置fq=36000/(3+1);
		can->MCR &= ~(1 << 0);   		  //退出初始化		
	}
	if (can_baud == 250)
	{
		//can在APB1系统总线上，因此最高时钟频率是36MHz,波特率=1/(正常位时间)，
		//正常位时间=1*tq+tq*(TS1[3:0]+1)+tq*(TS2[2:0]+1),tq=(BRP[9:0]+1)*tpclk,CAN波特率预设计成500K,
		//baud=250K,tq=BRP*(1/36000000);
		//建议时间份额为16，时间段1为11，时间段2为4，跳转宽度为3
		can->BTR |= 0 << 30;
		can->BTR |= 3 << 24;             //重新同步跳转宽为(3+1)tq
		can->BTR |= 11 << 16;            //时间段1为(11+1)tq
		can->BTR |= 4 << 20;             //时间段2为(4+1)tq
		can->BTR |= 7 << 0;              //设置fq=36000/(7+1);
		can->MCR &= ~(1 << 0);   		  //退出初始化		
	}
	while ((can->MSR & (1<<0)) == 1) 
	{
		i++;
		if (i >0xFFF0)   return;			//注意这里的时间不能太短否则会配置不成功
	}
	//滤波初始化
	can->FMR |= 1 << 0; 
    can->FA1R &= ~(1 << 0);      		//过滤器0不激活
	can->FS1R |= 1 << 0;  				//过滤器位宽是32位
	can->FM1R |= 0 << 0;               //过滤器0工作在标识符屏蔽模式
	can->FFA1R |= 0 << 0;              //过滤器关联到FIFO0中
	can->sFilterRegister[0].FR1 = 0x00000000;   //32位ID
	can->sFilterRegister[0].FR2 = 0X00000000;   //32位MASKER
	can->FA1R |= 1 << 0;                //激活过滤器0
	can->FMR &= 0 << 0;                 //过滤器工作在正常模式
	return  ;
}


void CAN_INT_ENBL(CAN_TypeDef* can) 
{
	if (can == CAN1) {
		can->IER |= 1 << 1;				//FIFO消息挂号中断允许//modify by zhoutao
        
    NVIC_InitTypeDef NVIC_InitStructure = { 
        .NVIC_IRQChannel = CAN1_RX1_IRQn,
        .NVIC_IRQChannelPreemptionPriority = 2,
        .NVIC_IRQChannelSubPriority = 2,
        .NVIC_IRQChannelCmd = ENABLE,
    };
    NVIC_Init(&NVIC_InitStructure);
    CAN_ITConfig(CAN1, CAN_IT_FF0, ENABLE);    
//	MY_NVIC_Init(2, 0, CAN1_RX1_IRQn, 2);  //CAN1_RX0_IRQn表示采用FIF0的接收中断
	
    }
}


void CAN_PRPH_Init(void)
{
	#ifdef CAN1_PRPH  
		/* PA11--CAN1_RX,PA12--CAN1_TX */
        RCC->APB2ENR |= 1 << 2;					//使能A端口时钟
        GPIOA->CRH &= 0XFFF00FFF;
        GPIOA->CRH |= 0X000B8000;      			//PA11配置成上拉输入模式,PA12配置成50MHz复用推挽输出模式
        GPIOA->ODR |= 3 << 11;          		//上拉
        /* CAN1外设位于APB1总线上 */
        RCC->APB1ENR |= 1 << 25;        //CAN时钟开启
        RCC->APB1RSTR |= 1 << 25;       //复位CAN模块
        RCC->APB1RSTR &= ~(1 << 25);    //退出复位模式

		CAN_FNCT_Init(CAN1, CAN1_BAUD);
        #ifdef CAN1_INT
            CAN_INT_ENBL(CAN1);
        #endif
	#endif
	
	#ifdef CAN2_PRPH
        /* PB12--CAN2_RX,PB13--CAN2_TX */
        RCC->APB2ENR |= 1 << 3;					//使能B端口时钟
        GPIOB->CRH &= 0XFF00FFFF;
        GPIOB->CRH |= 0X00B80000;      			//PB12配置成上拉输入模式,PB13配置成50MHz复用推挽输出模式
        GPIOB->ODR |= 3 << 11;          		//上拉
        /* CAN2外设位于APB1总线上 */
        RCC->APB1ENR |= 1 << 26;        //CAN2时钟开启
        RCC->APB1RSTR |= 1 << 26;       //复位CAN模块
        RCC->APB1RSTR &= ~(1 << 26);    //退出复位模式
        
	    CAN_FNCT_Init(CAN2, CAN2_BAUD);
        #ifdef CAN1_INT
		    CAN_INT_ENBL(CAN2);
	    #endif
	#endif
}






/*---------------------------------定义CAN线程栈------------------------------------*/
static struct rt_thread CAN1_send_thread;

/* 定义线程控栈时要求RT_ALIGN_SIZE个字节对齐 */
ALIGN(RT_ALIGN_SIZE)
/* 定义线程栈 */
static rt_uint8_t rt_CAN1_send_thread_stack[1024];

static void CAN1_send_thread_entry(void* parameter)
{
    uint16_t  i = 0;
    CAN_DATA_OPERATING CAN1_Fram_send;
    while (1) {
        if (CAN1_SEND_BUF.buf_lenth == 0) {
            return ;
        } else {
            CAN1_SEND_BUF.lock_flag = Locked; 
            for (i = CAN1_SEND_BUF.buf_lenth; i <= 0; i-- ) {
                CAN1_Fram_send = (CAN_DATA_OPERATING)CAN1_SEND_BUF.CAN_Send_buff[i];
                CAN_Send(CAN1, &CAN1_Fram_send);
                CAN1_SEND_BUF.buf_lenth = i;
            }
        }
    }
}

static void can_thread_init(void) 
{
    rt_thread_init(&CAN1_send_thread,                             //线程控制块
                "can1_send",                                  //线程名字
                CAN1_send_thread_entry,                       //线程入口函数
                RT_NULL,                                //线程入口参数
                &rt_CAN1_send_thread_stack[0],                //线程栈起始地址
                sizeof(rt_CAN1_send_thread_stack),            //线程栈大小
                3,                                      //线程优先级
                20);                                    //线程时间片
/*---------------------------------启动线程，开启调度-------------------------------------------*/
    rt_thread_startup(&CAN1_send_thread);
}










/* CAN1中断接收内部的数据 */
void  CAN1_RX0_IRQHandler(void)
{
	if (CAN1_RECV_BUF.lock_flag == 0) {
		CAN_Rx_Msg(CAN1, FIFOX0, &CAN1_RECV_BUF);
		CAN1_RECV_BUF.buf_lenth++;
		if (CAN1_RECV_BUF.buf_lenth >= CAN_RCV_BUF) {
			CAN1_RECV_BUF.buf_lenth = 0;
		}
	}
}
