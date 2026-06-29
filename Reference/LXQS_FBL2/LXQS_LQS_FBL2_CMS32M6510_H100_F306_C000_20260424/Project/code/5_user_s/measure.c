
//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    main.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   
*---------------------------------------------------------------------------//
*****************************************************************************/
//==========================================================================//


//---------------------------------------------------------------------------/
//	include files
//---------------------------------------------------------------------------/
#include "Header_Motor.h"
#include "Header_MCU.h"
#include "Header_User.h"
//---------------------------------------------------------------------------/
//	Local pre-processor symbols/macros('#define')
//---------------------------------------------------------------------------/

//---------------------------------------------------------------------------/
//	Local variable  definitions
//---------------------------------------------------------------------------/
Struct_Sample						Stru_Sample						= {0};					//采样AD值
Struct_Meas							Stru_Meas							= {0};					//测量结构体    
Struct_Coff							Stru_Coff							= {0};					//转化系数

//---------------------------------------------------------------------------/
//	Global variable definitions(declared in header file with 'extern')
//---------------------------------------------------------------------------/

//---------------------------------------------------------------------------/
//	Local function prototypes('static')
//---------------------------------------------------------------------------/


//===========================================================================/
//***** definitions  end ****************************************************/
//===========================================================================/

/*****************************************************************************
* Function Name  : HW_Sample_Init
* Description    : 采样数据初始化：偏置采样、BG校准、采样数据初始化
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Lsy
******************************************************************************/
int32_t Sumbuffer[7] = {0,0,0,0,0,0,0};
int32_t temp = 0;
void HW_Sample_Init(void)
{
	uint16_t Sampcnt =0;
	
	
//偏置采样时，先将IPD通道加入，偏置采样完成再将IPD通道取消
	ADC_EnableEPWMTriggerChannel(ADC_SCAN_IPD);	
	DelayTime_ms(2);

	for(Sampcnt = 0;Sampcnt < 64; Sampcnt++)
	{										
		Sumbuffer[0] += Get_ADC_Result(ADC_DATA_IBUS);
		Sumbuffer[1] += Get_ADC_Result(ADC_DATA_CHA);
		Sumbuffer[2] += Get_ADC_Result(ADC_DATA_CHB);
		Sumbuffer[3] += Get_ADC_Result(ADC_DATA_IPD);
		
		#if(Config_VOLTAGE_SAMPLE==VOLTAGE_SAMPLE_ENABLE)
		{
			Sumbuffer[4] += Get_ADC_Result(ADC_DATA_UBEMF);
			Sumbuffer[5] += Get_ADC_Result(ADC_DATA_VBEMF);
			Sumbuffer[6] += Get_ADC_Result(ADC_DATA_WBEMF);
		}
		#endif
	
		DelayTime_ms(1);		
	}
	
//偏置采样时，先将IPD通道加入，偏置采样完成再将IPD通道取消
	ADC_DisableEPWMTriggerChannel(ADC_SCAN_IPD);	
	

	Stru_Sample.IBusOffset	= Sumbuffer[0] >> 6;
	Stru_Sample.IaOffset 		= Sumbuffer[1] >> 6;
	Stru_Sample.IbOffset		= Sumbuffer[2] >> 6;
	Stru_Sample.IIPDOffset  = Sumbuffer[3] >> 6;
	Stru_Sample.BefUOffset  = Sumbuffer[4] >> 6;
	Stru_Sample.BefVOffset  = Sumbuffer[5] >> 6;
	Stru_Sample.BefWOffset  = Sumbuffer[6] >> 6;

	Stru_Sample.LHallAlphaOffset = LHall_Para[OFFSET_LHALLALPHA_INDEX];
	Stru_Sample.LHallBetaOffset = LHall_Para[OFFSET_LHALLBETA_INDEX];
	//---------------------------------------------------------------------------/
	// 偏置监测
	#if (OFFSET_PROTECT_ENABLE)
	{
		Fault_OFFSET_Check();
	}
	#endif

	//---------------------------------------------------------------------------/
	// BG校准
	#if (CONFIG_BANDGAP_MODE == BANDGAP_ENABLE)
	{
		// 使能内部通道22，设置为BG
		ADC_EnableEPWMTriggerChannel( ADC_SCAN_BG);
		ADC_SelAN22Source(ADC_AN22_SEL_BG);
		
		uint16_t i;
		uint32_t ADVal_Sum = 0;
		
		ADC_ConfigRunMode(ADC_MODE_HIGH,ADC_CONVERT_CONTINUOUS,ADC_CLK_DIV_1,45);	/* 36.5TADCK + 采样保持时间 */

		for(i = 0;i < 128;i++)
		{
			//开启ADC0转换
			ADC_Go();
			while(ADC_IS_BUSY());
			
			ADVal_Sum += ADC->DATA[ADC_DATA_BG];
			DelayTime_ms(1);		WDT_Restart();	
		}

		Stru_Sample.BG_AD = (ADVal_Sum >> 7);
		
		Stru_Sample.G_BG_Q12 =  (1.45 * _Q12_VAL *_Q12_VAL / (Stru_Sample.BG_AD *HW_ADC_REF));				// Q12  Vbg = 1.45V
		ADC_ConfigRunMode(ADC_MODE_HIGH,ADC_CONVERT_CONTINUOUS,ADC_CLK_DIV_1,25);
		
		//关闭通道
		ADC_DisableEPWMTriggerChannel( ADC_SCAN_BG);
		ADC_SelAN22Source(ADC_AN22_SEL_TS);
		
		// 折算转换系数
		Stru_Coff.AD2PU_Ibus		= (int32_t)(AD2Pu_Coeff_Ibus		* Stru_Sample.G_BG_Q12) >> 2;					// Q10
		Stru_Coff.AD2PU_Vbus		= (int32_t)(AD2Pu_Coeff_Vbus		* Stru_Sample.G_BG_Q12) >> 2;					// Q10
		Stru_Coff.AD2PU_Ip			= (int32_t)(AD2Pu_Coeff_Iphase	* Stru_Sample.G_BG_Q12) >> 2;					// Q10
		Stru_Coff.AD2PU_Vctrl		= (int32_t)(AD2Pu_Coeff_Vctrl		* Stru_Sample.G_BG_Q12) >> 2;					// Q10
	}
	#endif
	
	//---------------------------------------------------------------------------/
	// 采样数据初始化
	Stru_Sample.VbusAD = Get_ADC_Result(ADC_DATA_VBUS);
	Stru_Meas.PU_Value.Vbus = Stru_Sample.VbusAD * Stru_Coff.AD2PU_Vbus >> 10; 
	Stru_Meas.PU_Filt.Vbus  = Stru_Meas.PU_Value.Vbus;
	
//	Stru_Sample.MosTempAD = Get_ADC_Result(ADC_DATA_TEMP);
}

/*****************************************************************************
* Function Name  : Sample_MidFre
* Description    : 中频采样\转换
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建			Lsy
									 V0.2    2024/07/11   新增转换系数ModuSig2PU_Vphase			Wj
******************************************************************************/
void Sample_MidFre(void)
{
	//---------------------------------------------------------------------------/
	// 母线电压采样计算
  Stru_Sample.VbusAD 			= Get_ADC_Result(ADC_DATA_VBUS); 
	Stru_Meas.PU_Value.Vbus = Stru_Sample.VbusAD * Stru_Coff.AD2PU_Vbus >> 10;          
	Stru_Meas.PU_Filt.Vbus = LPF_Cal(&LPFVbus,Stru_Meas.PU_Value.Vbus,_1MS_LPF_10Hz);

	//---------------------------------------------------------------------------/
	// 母线电流采样计算
	Stru_Sample.IBusAD 			= Get_ADC_Result(ADC_DATA_IBUS);
	Stru_Meas.PU_Value.Ibus = (Stru_Sample.IBusAD - Stru_Sample.IBusOffset) * Stru_Coff.AD2PU_Ibus >> 10;   
	Stru_Meas.PU_Filt.Ibus  = LPF_Cal(&LPFIbus,Stru_Meas.PU_Value.Ibus, _1MS_LPF_10Hz);
	
	//---------------------------------------------------------------------------/
	// 功率计算
	Stru_Meas.PU_Value.Power 	= Stru_Meas.PU_Value.Ibus * Stru_Meas.PU_Value.Vbus >> 14;	
	//Stru_Meas.PU_Value.Power  = MC_SoftPower_Calc();			
	Stru_Meas.PU_Filt.Power = LPF_Cal(&LPFPower,Stru_Meas.PU_Value.Power, _1MS_LPF_10Hz);
	
	//---------------------------------------------------------------------------/
	//调速电压采样计算
	Stru_Sample.VspAD  				= Get_ADC_Result(ADC_DATA_CTRL);	
	Stru_Meas.PU_Value.Vctrl	= Stru_Sample.VspAD * Stru_Coff.AD2PU_Vctrl >> 10;	
	Stru_Meas.PU_Filt.Vctrl 	= LPF_Cal(&LPFVctrl,Stru_Meas.PU_Value.Vctrl, _1MS_LPF_5Hz);

	//---------------------------------------------------------------------------/
	// 温度信号采样	
	Stru_Sample.MosTempAD		= Get_ADC_Result(ADC_DATA_TEMP);	
	
	//---------------------------------------------------------------------------/
	// 观测器电压计算
	Stru_Coff.ModuSig2PU_Vphase = Stru_Coff.ModuSig2PU_Vphase_temp * Stru_Meas.PU_Filt.Vbus >>10;     //系数Q14
	
	

}

/*****************************************************************************
* Function Name  : PhyVal_Cal
* Description    : 物理值计算
* Function Call  : 主循环10ms任务
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/06/24    新建					Lsy
******************************************************************************/
void PhyVal_Cal(void)
{
	Stru_Meas.PhyValue.Vbus100mV  = Stru_Coff.PU2Phy_Vbus * Stru_Meas.PU_Filt.Vbus >> 15;
	Stru_Meas.PhyValue.Ibus10mA   = Stru_Coff.PU2Phy_Ibus * Stru_Meas.PU_Filt.Ibus >> 15;
	Stru_Meas.PhyValue.Power100mW = Stru_Coff.PU2Phy_Power * Stru_Meas.PU_Filt.Power >> 15;
	Stru_Meas.PhyValue.Vctrl_10mV = Stru_Coff.PU2Phy_Vctrl * Stru_Meas.PU_Filt.Vctrl >> 15;
}

/******************************** END OF FILE *******************************/




