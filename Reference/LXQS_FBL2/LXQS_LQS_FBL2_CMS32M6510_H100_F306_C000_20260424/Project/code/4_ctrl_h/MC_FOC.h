//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    main.h
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   
*---------------------------------------------------------------------------//
*****************************************************************************/

//==========================================================================//

#ifndef __FOC_H
#define __FOC_H

/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>
#include "MC_PID.h"
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/


/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/

typedef struct
{
    int32_t Ia;
    int32_t Ib;
    int32_t Ic;
}Struct_Cur_abc;

typedef struct
{
    int32_t Ialpha;
    int32_t Ibeta; 
}Struct_Cur_alphabeta;

typedef struct
{
    int32_t Id;
    int32_t Iq;
}Struct_Cur_dq;


typedef struct
{
    int32_t Ua;
    int32_t Ub;
    int32_t Uc;  
}Struct_Vol_abc;

typedef struct
{
    int32_t Ualpha;
    int32_t Ubeta; 
}Struct_Vol_alphabeta;

typedef struct
{
    int32_t Ud;
    int32_t Uq; 
}Struct_Vol_dq;

typedef struct
{
    int32_t Sin;
    int32_t Cos; 
}Struct_Sincos;


typedef struct
{
		int16_t LHall_LearnState;				// 线性霍尔学习状态
    int16_t Elec_Theta;							// 转子位置
    int32_t OffsetTheta;						// 补偿角度
    int32_t Elec_Omega;							// 角速度，锁相环值
    int32_t OmegaPU;								// 角速度的标幺值
    int32_t OmegaPUFiltered;			// 滤波后的角速度，锁相环值
	
		int32_t	IPD_Theta;							// IPD定位角度
	
    int32_t Vsmax;									// 电压矢量最大值
    int32_t VsmaxSquare;   					// 电压矢量最大值平方
		int32_t Q15_VsmaxSquare;				// 电压矢量最大值平方Q15
		int32_t VsAmp;									// 实际电压矢量长度
	
    int32_t WeakThreshold;					// 弱磁阈值
		int32_t WeakVs_Square;						// 弱磁阈值平方Q15	
	
    int32_t EPWM_Period;						// EPWM的周期值
	
		int32_t Curr_Is_Ref;						// 电流矢量给定值
		int32_t Curr_Is_Max;						// 电流矢量（相电流）最大值
		int32_t Curr_Is_Min;						// 电流矢量（相电流）最小值
	
}Struct_Foc;


typedef struct
{
	int16_t ThetaM;							//机械角度
	int16_t Offset_ThetaM;			//机械角度的偏置
	
	
	int16_t ThetaE;							//电角度
	uint8_t Pairs;							//极对数
	
	Struct_Sincos SinCosSig;		//
	Struct_Sincos Sincos;				//
	int32_t Omega;
	int32_t OmegaPU;
	int32_t OmegaPUFiltered;
	
	
	
	int16_t ThetaEPLL;					//电角度的锁相环值
	Struct_PI pi;
}Struct_Encoder;





typedef struct
{
  uint32_t CntCaloffset;
  uint32_t CntIdle;
  uint32_t CntInit;
  uint32_t CntCharge;
  uint32_t Cnttesthd;
  uint32_t CntWind;
  uint32_t CntAlign1;
  uint32_t CntAlign2;
  uint32_t CntStartup1;
  uint32_t CntStartup2;
  uint32_t CntSw;
  uint32_t CntRun;
  uint32_t CntStop;
	uint32_t CntBrake;
  uint32_t CntFault;
	
	
}Struct_FOCCount;                                  //计数器结构体


typedef enum
{
		MC_POWERON         = 0,                        //上电
	  MC_LHALL_LEARN     = 1,												 //线性霍尔学习
		MC_CONVERGENCE     = 2,												 //霍尔收敛
    MC_IDLE            = 3,                        //空闲                    
    MC_INIT            = 4,                        //初始化
    MC_TEST            = 5,                        //硬件测试模式 
		MC_CHARGE          = 6,                        //自举充电 
    MC_WIND            = 7,                        //顺逆风
    MC_IPD             = 8,                        //锁定
    MC_ALIGN           = 9,                        //锁定
    MC_STARTUP         = 10,                       //启动
    MC_SW              = 11,                       //状态切换
    MC_RUN             = 12,                       //运行态
    MC_STOP            = 13,                       //停止                          
    MC_BRAKE           = 14                        //故障                 
}MotorState_e; 
    
typedef struct
{
	int32_t K1;                                //观测器参数1         
  int32_t K2;                                //观测器参数2
  int32_t PLLKp;                             //锁相环比例系数
  int32_t PLLKi;                             //锁相环积分系数
	
	uint8_t q;
	uint8_t m;
	
}Struct_LibOb1;                              //观测器1库参数结构体


typedef struct
{
	int32_t K1;                                //观测器参数1         
  int32_t K2;                                //观测器参数2
  int32_t PLLKp;                             //锁相环比例系数
  int32_t PLLKi;                             //锁相环积分系数
	
	uint8_t q;
	uint8_t m;
}Struct_LibOb2;                              //观测器2库参数结构体

typedef struct
{
	int32_t OmegaPu;													 //最低滤波频率，标幺值
  int32_t PLLKp;                             //锁相环比例系数
  int32_t PLLKi;                             //锁相环积分系数
	
	uint8_t q;
	uint8_t m;
}Struct_LibOb3;                              //观测器3库参数结构体


typedef struct
{
  int32_t TsPu;                              //时间基值
  int32_t AdpMax;                            //最大自适应律
  int32_t AdpMin;                            //最小自适应律 
  int32_t OmegaPLL2SpdPUQ15;                 //转化系数Q8，将角速度的锁相环值转化成标幺值的Q15格式
  int32_t ElecOmegaBase;                     //电角频率基值
	
	Struct_LibOb1 LibOb1;											 //观测器1需要的库参数
	Struct_LibOb2 LibOb2;											 //观测器2需要的库参数
	Struct_LibOb3 LibOb3;											 //观测器3需要的库参数

}Struct_AlgorPara;      //算法库参数


typedef struct
{

	uint16_t		ACCCycle;					// 加速间隔周期
	int16_t			DeataCur;

	int16_t			OmegaEMax;					//    拖动角速度
	int16_t			OmegaE;	
	
	int16_t			DragCurr;	
	int32_t			OpenCurr_Final;

	
}Struct_Test;                                  //计数器结构体


/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/


extern Struct_Cur_abc            Stru_Cur_abc;                 //Ia、Ib、Ic的标幺值的Q15格式      
extern Struct_Cur_alphabeta      Stru_Cur_alphabeta;           //Ialpha、Ibeta的标幺值的Q15格式
extern Struct_Cur_dq             Stru_Cur_dqRef;               //dq轴参考电流的标幺值的Q15格式
extern Struct_Cur_dq             Stru_Cur_dq;                  //dq轴反馈电流的标幺值的Q15格式

extern Struct_Vol_abc            Stru_Vol_abc;                 //三相电压Ua、Ub、Uc的标幺值的Q15格式
extern Struct_Vol_alphabeta      Stru_Vol_alphabeta;           //Ualpha、Ubeta的标幺值的Q15格式
extern Struct_Vol_alphabeta      Stru_Vol_alphabeta_Comp;
extern Struct_Vol_dq             Stru_Vol_dq;                  //dq轴电压的标幺值的Q15格式
extern Struct_Sincos             Stru_Sincos;                  //转子位置的正余弦值


extern Struct_Vol_alphabeta      Stru_Vol_alphabetaSample;		 //通过反电动势采样电路得到的Ua、Ub、Uc的调制信号的Q15格式
extern Struct_Vol_abc            Stru_Vol_abcSample;				   //通过反电动势采样电路得到的Ualpha、Ubeta的调制信号的Q15格式



extern Struct_Foc                Stru_Foc;                     //Foc的相关参数
extern MotorState_e              MOTOR_STATE;       					 //电机状态机
extern Struct_FOCCount           Stru_FocCnt;                   //状态机计数器
extern Struct_AlgorPara          Stru_AlgorPara;               //算法库参数
extern uint16_t 	               VSData[4];              			 //串口发送数据缓存

extern Struct_Test							Stru_Test;
extern Struct_Encoder						Stru_Encoder;									//编码器结构体 
/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/

Struct_Sincos SinCos_Cal(int16_t angle);
void FOC_Task_MidFre(void);
void FOC_Task_HighFre(void);
void RevPark(void);
void Park(void);
void FOC_VsLimit(void);
void Cal_Slope(void);
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/

#endif

/******************************** END OF FILE *******************************/
