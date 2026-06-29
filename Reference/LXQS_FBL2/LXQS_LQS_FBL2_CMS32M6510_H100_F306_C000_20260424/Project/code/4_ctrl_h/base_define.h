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

#ifndef __BASE_H
#define __BASE_H

/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>
#include "Set_HW.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/
#define DEG_Minus180 	(-32768)
#define DEG_Minus150 	(-27306)
#define DEG_Minus120 	(-21845)
#define DEG_Minus90 	(-16384)
#define DEG_Minus60 	(-10922)
#define DEG_Minus30 	(-5461)
#define DEG_0   			(0)
#define DEG_15  			(2730)
#define DEG_30  			(5461)
#define DEG_60  			(10922)
#define DEG_90  			(16384)
#define DEG_120 			(21845)
#define DEG_150 			(27306)
#define DEG_180 			(32767)

#define 	_2PI()											(6.283185307)	
#define 	_PI()												(3.1415926535)	
#define 	_SQRT_3											(1.732051)
#define 	_SQRT_2											(1.414214)
#define 	_Q16_VAL										(65535)
#define 	_Q15_VAL										(32768)	
#define 	_Q14_VAL										(16384)	
#define 	_Q12_VAL										(4096)	
#define 	_Q10_VAL										(1024)	
	

#define 	_Q10_105PCT										(1075)	
/*****************************************************************************/

//--------------------------------基值-------------------------------------------/
//交流侧基值： 相电流基值
#define  I_PHASE_BASE        					((float)(HW_ADC_REF-HW_OFFSET_IPHASE) / (HW_RSHUNT_IPHASE*HW_GAIN_IPHASE))     
//直流侧基值： 母线电流基值	
#define  I_DC_BASE           			    ((float)P_DC_BASE / V_DC_BASE)							

//--------------------------------AD到标幺转换系数-------------------------------------------/
#define  AD2Pu_Coeff_Vbus     		    ((float)((HW_ADC_REF*8) /(V_DC_BASE*HW_VBUS_DIVIDER*2.0)) )										
#define  AD2Pu_Coeff_Ibus						  ((float)((HW_ADC_REF*8) /(HW_RSHUNT_IBUS*HW_GAIN_IBUS*I_DC_BASE*2.0)) )			
#define  AD2Pu_Coeff_Iphase						((float)((HW_ADC_REF*8) /(HW_ADC_REF - HW_OFFSET_IPHASE)) )												
#define  AD2Pu_Coeff_Vctrl						( 8 )		

#define  AD2ModuSig_Coeff_BEMF							((float)((HW_ADC_REF*8) /((V_DC_BASE / _SQRT_3)*HW_BEMF_DIVIDER)) )												
//--------------------------------物理值到标幺转换系数-------------------------------------------/
#define  Phy2Pu_Coeff_Vbus						((float)( _Q14_VAL /V_DC_BASE))
#define  Phy2Pu_Coeff_Ibus      		  ((float)( _Q14_VAL /I_DC_BASE))
#define  Phy2Pu_Coeff_Power   		    ((float)( _Q14_VAL /P_DC_BASE))
#define  Phy2Pu_Coeff_Iphase					((float)( _Q15_VAL /I_PHASE_BASE))
#define  Phy2Pu_Coeff_Speed						((float)( _Q14_VAL /MECH_SPEED_BASE))
#define  Phy2Pu_Coeff_Vctrl						((float)( _Q15_VAL /HW_ADC_REF))
//--------------------------------物理值到标幺转换宏定义函数-------------------------------------------/
#define  Phy2Pu_Fun_Vbus(V)						(int32_t)(V *_Q14_VAL /V_DC_BASE)																		// 	母线电压标幺值计算	目标Q14
#define  Phy2Pu_Fun_Ibus(A)						(int32_t)(A *_Q14_VAL /I_DC_BASE)  																	//  母线电流标幺值计算	目标Q14
#define  Phy2Pu_Fun_Power(P)					(int32_t)(P *_Q14_VAL /P_DC_BASE)  																  //  母线功率标幺值计算	目标Q14
#define  Phy2Pu_Fun_Iphase(A)					(int32_t)(A *_Q15_VAL /I_PHASE_BASE)  															//  相电流标幺值计算		目标Q15
#define  Phy2Pu_Fun_Vctrl(V)					(int32_t)(V *_Q15_VAL /HW_ADC_REF)																	//  调速电压标幺值计算	目标Q15

#define  Phy2Pu_Fun_Speed(S)					(int32_t)(S *Phy2Pu_Coeff_Speed)																		//  转速标幺值计算			目标Q14

//--------------------------------标幺值到物理值转换系数-------------------------------------------/
#define  Pu2Phy_Coeff_Vbus						(2 * 10 * V_DC_BASE )										// VbusPU  * Pu2Phy_Coeff_Vbus >> 15 = PhyVal   100mV
#define  Pu2Phy_Coeff_Ibus						(2 * 100* I_DC_BASE )										// IbusPU  * Pu2Phy_Coeff_Vbus >> 15 = PhyVal  	10mA
#define  Pu2Phy_Coeff_Power						(2 * 10 * P_DC_BASE )										// PowerPU * Pu2Phy_Coeff_Vbus >> 15 = PhyVal		100mW
#define  Pu2Phy_Coeff_Iphase					(100 * I_PHASE_BASE )										// IphasePU* Pu2Phy_Coeff_Vbus >> 15 = PhyVal   10mA
#define  Pu2Phy_Coeff_Vctrl						(100 * HW_ADC_REF )											// Vctrl   * Pu2Phy_Coeff_Vbus >> 15 = PhyVal   10mV	
//--------------------------------物理值计算-------------------------------------------/
#define	 MOTOR_PSI										(3 * MOTOR_KE / 100/MOTOR_PAIRS/3.141592653)      // 永磁体磁链 单位Wb

#define  Coeff_OmegaPLL2MecSpd				((int32_t)(EPWM_FREQ * 30 / MOTOR_PAIRS))					//_Q15_VAL *EPWM_FREQ * 60 / MOTOR_PAIRS/_Q16_VAL

/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/




/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/


/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/



/*****************************************************************************/
/* Intrinsic function definition ('extern', definition in C source) */
/*****************************************************************************/

/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/




#endif

/******************************** END OF FILE *******************************/





