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

#ifndef __SOFTINIT_H
#define __SOFTINIT_H

/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/

#define	Flag_MotorDir				B_Flag.Bits.MotorFR
#define	Flag_1ms_Intr				B_Flag.Bits.Intr_1ms
#define	Flag_FOC_Intr				B_Flag.Bits.Intr_Foc


#define 							LHall_Para_Length 							(6)	

#define 							LEARN_STATE_INDEX								(0)
#define 							LHALL_ORDER_INDEX								(1)
#define 							OFFSET_LHALLALPHA_INDEX					(2)
#define 							OFFSET_LHALLBETA_INDEX					(3)
#define 							LHALL_OFFSET_ANGLE_INDEX				(4)
#define 							CHECKSUM_INDEX									(5)

#define							 LEARN_FINISHED										(0x1111)
#define							 LEARN_ING												(0x2222)

#define HALL_SEQ_FORWARD    (0x1234)
#define HALL_SEQ_REVERSE    (0x4321)

/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/

typedef struct 
{
//电机侧基值
    float Iphase;                 //电流基值
    float MechSpd;                //机械转速基值
    float Vphase;                 //相电压基值
	
    float ElecSpd;                //电转速基值
    float Rs;                     //相电阻基值
    float MechOmega;              //机械角速度基值
    float ElecOmega;              //电角速度基值
    float Psi;                    //磁链基值
    float Ls;                     //电感基值
    float Te;                     //转矩基值
    float Ts;                     //开关周期基值
    float J;                      //转动惯量基值
//母线侧基值
    float Vdc;                    //母线电压基值
		float Ibus;									  //母线电流基值
}Struct_BaseValue;


typedef struct 
{
    uint8_t pole;                      //电机极对数                  
    int32_t RsPU;                      //定子电阻标幺值
    int32_t LdPU;                      //d轴电感标幺值
    int32_t LqPU;                      //q轴电感标幺值
    int32_t PsiPU;                     //永磁磁链标幺值
    int32_t Jpu;                       //转动惯量标幺值     
}Struct_Motor;


typedef struct 
{
  int32_t AlignCurMin;            //对齐电流初始值
  int32_t AlignCurMax;            //对齐电流最终值
  int16_t AlignCurInc;            //对齐电流增量 
  int16_t AlignCurIncInterval;    //对齐电流增加间隔
  int32_t AlignTime;              //对齐时间
  
  int32_t StartCurMin;            //启动电流初始值
  int32_t StartCurMax;            //启动电流最终值
  int16_t StartCurIncInterval;    //对齐电流增加间隔
  int16_t StartCurInc;            //电流增量 
  

  
  int32_t PLLKp;        //锁相环比例系数
  int32_t PLLKi;        //锁相环积分系数
	
	int32_t Adp_Max;				//最大自适应率
	int32_t Adp_Min;				//最小自适应率

  
  int32_t Speed_Close;     //闭环转速 真实值
  int32_t HoldTime;        //闭环保持时间
	int32_t SwitchTime;			 //SW状态机保持时间
  

}Struct_Start;



typedef struct 
{
  int32_t Dkp;            //D轴电流环比例系数
  int32_t Dki;            //D轴电流环积分系数
  int32_t Dout_Max;       //D轴电流环最大输出
  int32_t Dout_Min;       //D轴电流环最小输出
  
  int32_t Qkp;            //Q轴电流环比例系数
  int32_t Qki;            //Q轴电流环积分系数
  int32_t Qout_Max;       //Q轴电流环最大输出
  int32_t Qout_Min;       //Q轴电流环最小输出
  
  int32_t PLLKp;          //锁相环比例系数
  int32_t PLLKi;          //锁相环积分系数
  
  int32_t PwrKp;          //功率环比例系数
  int32_t PwrKi;          //功率环积分系数

  int32_t SpdKp;          //速度环比例系数
  int32_t SpdKi;          //速度环积分系数
  int32_t SpdKd;          //速度环微分系数
	
  int32_t WeakFluxKp;          //弱磁环Kp
  int32_t WeakFluxKi;          //弱磁换Ki
	int32_t WF_AngMax;          //弱磁最大角度
}Struct_Run;


typedef struct 
{
  int32_t Vbus;            //Vbus 滤波系数 
  int32_t Ibus;            //Ibus 滤波系数 	
	
  int32_t Vctrl;           //Ibus 滤波系数 	
	int32_t Power;
	int32_t Speed;
	
	int32_t	MotorOmegaE;		// 观测器We

}Struct_LPF;

//
typedef struct 
{
  uint8_t SVPWM;        	//0代表5段式   1代表7段式
  uint8_t Shunt;        	//0代表单电阻  1代表双电阻
  uint8_t Dir;          	//0代表逆时针  1代表顺时针
	uint8_t Start;					// 0：方波启动	1：霍尔启动		2：观测器1启动	3：观测器2启动
	uint8_t RunOb;					// 1：观测器1		2：观测器2  	3：观测器3
	

}Struct_Mode;


//
typedef struct 
{
  Struct_Start		Start;    	//启动参数
  Struct_Run			Run;      	//运行参数
  Struct_Mode			Mode;     	//运行模式参数
	Struct_LPF			LPFCoff;
}Struct_Para;

typedef struct
{
	uint32_t			PowerOn;
	uint32_t	 		Motor_Restart;
	uint32_t	 		Motor_PowerDown;
	uint32_t	 		Motor_Brake;	
	
	uint32_t	 		Sleep;		
	
}struct_Time;

typedef	union
{
	uint16_t Byte;
	struct
	{
		uint16_t	MotorFR		:1;								// 当前运行方向
		uint16_t	Intr_Foc	:1;								// FOC中断标志
		uint16_t	Intr_1ms	:1;								// 1ms中断标志
		uint16_t	Bit3	:1;
		uint16_t	Bit4	:1;
		uint16_t	Bit5	:1;
		uint16_t	Bit6	:1;
		uint16_t	Bit7	:1;
	}Bits;
}ByteFlag;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/
extern Struct_Para               				Stru_Para;
extern Struct_BaseValue                 Stru_BaseValue;
extern Struct_Motor                     Stru_Motor;                   //电机参数的标幺值的Q15格式
extern ByteFlag 												B_Flag;
extern struct_Time											Stru_Time;
extern int32_t		LHall_Para[LHall_Para_Length];
/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/

void BaseValue_Init(void);        //基值参数初始化
void Motor_Init(void);            //电机参数初始化
void Coff_Init(void);             //转化系数初始化
void User_init(void);             //用户命令初始化
void SetPara_init(void);          //用户设定参数初始化
void AlgorPara_Init(void);        //库参数初始化
void Foc_Para_Init(void);         //Foc参数初始化
void Foc_Ctrl_Init(void);         //Foc参数初始化
void LHall_Ctrl_Init(void);       //霍尔初始化
void MC_Para_Init(void);

void OpenLoop_ParaInit(void);

void VbusRippleComp_Para_Init(void);		
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/

#endif


/******************************** END OF FILE *******************************/

