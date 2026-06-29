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

#ifndef __MEASURE_H
#define __MEASURE_H


/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/


/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/
typedef struct
{
    int32_t VbusAD;

    int32_t IaAD;
    int32_t IaOffset;
    
    int32_t IbAD;
    int32_t IbOffset;
    
    int32_t IcAD;
    int32_t IcOffset;
    
    int32_t BefUAD;
    int32_t BefUOffset;
    
    int32_t BefVAD;
    int32_t BefVOffset;
    
    int32_t BefWAD;
    int32_t BefWOffset;
	
    int32_t IBusAD;
    int32_t IBusOffset;
	
    int32_t IIPDAD;
    int32_t IIPDOffset;
	
    int32_t HallSumAD;
    int32_t HallSumOffset;
    
    int32_t LHallAlphaAD;
    int32_t LHallAlphaOffset;
    
    int32_t LHallBetaAD;
    int32_t LHallBetaOffset;
    
    int32_t VspAD;
    int32_t VspOffset;
    
    int32_t MotorTempAD;
    int32_t MotorTempOffset;
    
    int32_t MosTempAD;
    int32_t MosTempOffset;

    int32_t HeaterTempAD;
    int32_t HeaterTempOffset;
		
		int32_t	BG_AD;
		int32_t	G_BG_Q12;
    
}Struct_Sample;


typedef struct
{
    int32_t Vbus100mV;
    int32_t Ibus10mA;
    int32_t MecSpeed;   
    int32_t Power100mW;
		int16_t	Vctrl_10mV;
		int16_t TempMos;

}Struct_PhyValue;                    //都是滤波之后的物理值


typedef struct
{
    int32_t Vbus;
    int32_t Ibus;
    int32_t MecSpd;                 
		int32_t Power;
		int32_t Vctrl;
		int32_t TempMos;
}Struct_PU_Value;


typedef struct
{
    Struct_PU_Value  PU_Value;  	 
		Struct_PU_Value  PU_Filt;
    Struct_PhyValue  PhyValue;
}Struct_Meas;

typedef struct
{
		//-----------AD值到标幺值Q10--------------------/
    int32_t AD2PU_Vbus; 	
		int32_t AD2PU_Ibus; 	
    int32_t AD2PU_Ip; 	
		int32_t	AD2PU_Vctrl;
	
		//------------反电势AD到调制信号转换系数Q10----------------/
		int32_t	AD2ModuSig_BEMF;			

		//----------物理值到标幺转换系数Q10--------------------/
		int32_t Phy2PU_Vbus; 
		int32_t Phy2PU_Ibus; 
		int32_t Phy2PU_Power; 

		int32_t Phy2PU_Iphase; 
		int32_t Phy2PU_Speed; 	
		
		int32_t	Phy2PU_Vctrl;
	
		//----------标幺值到物理值Q15---------------------- 
		int32_t PU2Phy_Vbus; 
		int32_t PU2Phy_Ibus; 
		int32_t PU2Phy_Power; 	
		
		int32_t PU2Phy_Iphase; 
		
		int32_t PU2Phy_Vctrl; 	

		//----------工作电压与观测器电压转换系数----------------------
		int32_t ModuSig2PU_Vphase_temp;		//缓存变量Q10,用于计算系数：ModuSig2PU_Vphase
		int32_t ModuSig2PU_Vphase;				//转化系数Q14，将程序中的Valpha Vbeta转化为标幺值的Q15格式
		int32_t AD2ModuSig_Bef_temp;		  //缓存变量,  用于计算系数:AD2ModuSig_Bef
		int32_t AD2ModuSig_Bef;		        //转化系数，将采样到的反电动势转化为调制信号


		//----------电角速度的锁相环值到机械角速度的转化系数----------------------
		int32_t	OmegaePLL2MecSpeed;     
	
}Struct_Coff;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/
extern Struct_Meas               Stru_Meas;                    //测量结构体    
extern Struct_Sample             Stru_Sample;                  //采样AD值
extern Struct_Coff               Stru_Coff;                    //转化系数

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/
void Sample_MidFre(void);
void HW_Sample_Init(void);
void PhyVal_Cal(void);
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/

#endif



/******************************** END OF FILE *******************************/


