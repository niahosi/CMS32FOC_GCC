
/*****************************************************************************/
//---------------------------------------------------------------------------//
/** \file
*  File Name  : Set_HW.h
*  Author     : CMS_Motor_Control_Team
*  Version    : v300
*  Date       : 2024/06/06
*  Description: 本页参数为硬件相关宏定义    
								包括但不限于ADC通道、载波频率、EPWM重映射等
//---------------------------------------------------------------------------//
******************************************************************************/
#ifndef __SET_HARDWARE_H
#define __SET_HARDWARE_H

#include "Set_FOC.h"


//============================================================================/
// 代码对齐方式：Edit--Configuration--Editor--C/C++Files--Tab size = 2
//----------------------------------------------------------------------------/

// 上电延时
#define   POWERON_DELAY_TIME										(1000)    //上电延时

//-------------------------------电流采样方式选择------------------------------/                                                                                                                                                               
#define   Single_Shunt													(1)											// 单电阻                  
#define   Double_Shunt													(2)											// 双电阻   
#define   Config_Shunt_Mode                     (Double_Shunt)    		

//-------------------------------采样通道配置------------------------------/
// 相电流
#if (Config_Shunt_Mode == Single_Shunt)

#define		ADC_DATA_CHA													(ADC_CH_0)
#define		ADC_SCAN_CHA													(ADC_CH_0_MSK)					// 相电流采样	

#define   ADC_DATA_CHB													(ADC_CH_1)
#define   ADC_SCAN_CHB													(ADC_CH_1_MSK)					// 相电流采样	

#define   HW_GAIN_IPHASE												(10.0)										// 相电流采样放大倍数                        
#define   HW_GAIN_IBUS                        	(10.0)										// Ibus电流采样放大倍数        

#define   HW_OFFSET_IPHASE                    	(0.8)										// 相电流采样偏置电压 (V)            
#define   HW_OFFSET_IBUS                      	(0.8)										// 母线电流采样偏置电压 (V)                      
																																																									
#define   HW_RSHUNT_IPHASE                    	(0.02)									// 相采样电阻 单位：Ω
#define   HW_RSHUNT_IBUS                      	(0.02)									// 母线采样电阻 单位：Ω



#elif (Config_Shunt_Mode == Double_Shunt)

#define   ADC_DATA_CHA													(ADC_CH_2)			
#define   ADC_SCAN_CHA													(ADC_CH_2_MSK)					// 相电流采样	

#define   ADC_DATA_CHB													(ADC_CH_3)
#define   ADC_SCAN_CHB													(ADC_CH_3_MSK)					// 相电流采样

#define   HW_GAIN_IPHASE                      	(10.0)										// 相电流采样放大倍数                        
#define   HW_GAIN_IBUS                        	(10.0)										// Ibus电流采样放大倍数        

#define   HW_OFFSET_IPHASE                    	(HW_ADC_REF/2.0)				// 相电流采样偏置电压 (V)            
#define   HW_OFFSET_IBUS                      	(0.8)										// 母线电流采样偏置电压 (V)                      
																																																									
#define		HW_RSHUNT_IPHASE											(0.02)									// 相采样电阻 单位：Ω
#define		HW_RSHUNT_IBUS												(0.05)									// 母线采样电阻 单位：Ω

#endif
//---------------------------------------------------------------------------//
// 相电压
#define   VOLTAGE_SAMPLE_DISABLE								(0)											// 端压采样不使能                 
#define   VOLTAGE_SAMPLE_ENABLE									(1)											// 端压采样使能     
#define   Config_VOLTAGE_SAMPLE                 (VOLTAGE_SAMPLE_DISABLE)    		

#if(Config_VOLTAGE_SAMPLE==VOLTAGE_SAMPLE_ENABLE)
	#define   ADC_DATA_UBEMF												(ADC_CH_4)
	#define   ADC_SCAN_UBEMF												(ADC_CH_4_MSK)					// MCU_BEMF_U

	#define   ADC_DATA_VBEMF												(ADC_CH_5)
	#define   ADC_SCAN_VBEMF												(ADC_CH_5_MSK)					// MCU_BEMF_V

	#define   ADC_DATA_WBEMF												(ADC_CH_6)
	#define   ADC_SCAN_WBEMF												(ADC_CH_6_MSK)					// MCU_BEMF_W
#endif

//---------------------------------------------------------------------------//
// 线性霍尔
#if(Config_Run_Mode==RUN_MD_LHALL)
	#define   ADC_DATA_LHALLAlpha										(ADC_CH_4)
	#define   ADC_SCAN_LHALLAlpha										(ADC_CH_4_MSK)					// HALL alpha

	#define   ADC_DATA_LHALLBeta										(ADC_CH_5)
	#define   ADC_SCAN_LHALLBeta										(ADC_CH_5_MSK)					// HALL beta
#endif

#define   ADC_DATA_IBUS													(ADC_CH_10)
#define   ADC_SCAN_IBUS													(ADC_CH_10_MSK)					// 母线电流采样

#define   ADC_DATA_VBUS													(ADC_CH_6)
#define   ADC_SCAN_VBUS													(ADC_CH_6_MSK)          // 母线电压采样

#define   ADC_DATA_CTRL													(ADC_CH_13)
#define   ADC_SCAN_CTRL													(ADC_CH_13_MSK)					// 调速信号采样

#define   ADC_DATA_TEMP													(ADC_CH_5)
#define   ADC_SCAN_TEMP													(ADC_CH_5_MSK)          // NTC温度信号采样


//---------------------------------------------------------------------------//
// 以下通道不可修改
#define   ADC_DATA_BG														(ADC_CH_22)
#define   ADC_SCAN_BG														(ADC_CH_22_MSK)					// Bandgap 采样

#define 	ADC_DATA_IPD       		                (ADC_CH_0)
#define 	ADC_SCAN_IPD           		            (ADC_CH_0_MSK)					// IPD电流采样通道


//-------------------------------参考电压配置------------------------------/
#define		ADCREF_3V6														(0)											// 使用内部LDO 3.6V              
#define		ADCREF_4V2														(1)											// 使用内部LDO 4.2V
#define		ADCREF_VCC														(2)											// 使用VCC供电 
#define		CONFIG_LDO														(ADCREF_VCC)						// ADC参考电压源选择
#define		HW_ADC_REF														(5.0)										// ADC参考电压 (V)  
//-------------------------------频率、载波配置------------------------------/
#define		MCU_CLK																(64000000ul)						// MCU CLK 
#define		EPWM_FREQ															(15000)									// EPWM FREQ 
#define		EPWM_Tus															(64)                        
#define		EPWM_DT																(0.5) 				          // 死区时间，64；0.5us 
#define		EPWM_PERIOD														(MCU_CLK/EPWM_FREQ/2)		// EPWM 周期寄存器值 
#define		EPWM_US																(1.0/EPWM_FREQ)					// 开关周期   单位s
#define		EPWM_HALFPERIOD												(EPWM_PERIOD/2)					// 半周期值
//-------------------------------电流采样时间参数---------------------------/
#define		EPWM_CPMTG														(4.5)										// 双电阻方案：采样提前时间 单位：us 

#define		EPWM_AHEAD														(0.5)										// 单电阻方案：提前采样时间 单位：us
#define		SAMP_STEADY														(2.0)										// 单电阻方案：信号（震荡）稳定时间，us
//-------------------------------母线工作电压-------------------------------/
#define		V_DC_BASE															(14.0)									// 母线额定工作电压
#define		HW_VBUS_DIVIDER												(2.0/12.0)							// 母线电压采样，分压电阻比   
#define		P_DC_BASE															(50.0)									// 功率基值 (W) 

//-------------------------------反电势监测电路-------------------------------/
#define		HW_BEMF_DIVIDER												(4.7/44.7)							// 反电势电路采样，分压电阻比   

//-------------------------------比较器配置-----------------------------------/
// 配置说明：使用母线硬件过流需使能ACMP_CH1
#define		ACMP_CH0															(0)
#define		ACMP_CH1															(1)
#define		ACMP_NONE															(2)
#define		ACMP_CH01															(3)
#define		ACMP_CH																(ACMP_CH1)	

//-------------------------------EPWM端口重映射配置---------------------------/
// 配置说明：按照硬件原理图中，MCU引脚EPWM 0-5 与 WH、VH、UH、WL、VL、UL的匹配关系填写
#define		IO_EPWM5															(WL)
#define		IO_EPWM4															(WH)
#define		IO_EPWM3															(VL)
#define		IO_EPWM2															(VH)
#define		IO_EPWM1															(UL)
#define		IO_EPWM0															(UH)

//-------------------------------双电阻采样顺序配置---------------------------/
// 配置说明：
// 双电阻方案：选择PGA1与PGA2 对应的采样通道顺序 如：PGA1采样W相电流，PGA2采样U相电流，则选择IP_WU
// 单电阻方案：无需配置
#define		IP_UV																	(0)
#define		IP_UW																	(1)
#define		IP_VU																	(2)
#define		IP_VW																	(3)
#define		IP_WU																	(4)
#define		IP_WV																	(5)
#define		IP_SAMP_CH                         		(IP_UV)

#endif

/******************************** END OF FILE *******************************/

