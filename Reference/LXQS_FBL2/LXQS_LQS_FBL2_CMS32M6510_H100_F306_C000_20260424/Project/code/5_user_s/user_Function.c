

//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    User.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   用于放置用户层的功能函数
*					 包括但不限于LED控制、485通信功能、数码管显示等待
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
*-----------------------------------------------------------------------------
* Function Name  :User_Sleep_Manage
* Description    :睡眠管理
* Function Call  :主函数调用
* Input Paragram :无
* Return Value   :无
*-----------------------------------------------------------------------------
******************************************************************************/
void User_Sleep_Manage(void)
{	
	if ((MOTOR_STATE == MC_IDLE) && (Stru_User.SigOnFlag == 0) )
	{
		if (++Stru_Time.Sleep >= 30)				//100*30 = 3s
		{
			Stru_Time.Sleep = 0;
			//关管
			Bridge_Output_Off();
			
			//关闭中断
			NVIC->ICER[0] = 0xFFFFFFFF;
			//特殊处理tick
			SysTick->CTRL  = 0x00000005;
		//***********************************************//	
			//设置部分引脚为输入高阻态
		//	GPIO_Init(PORT0,PIN0,ANALOG_INPUT);
		//**********************************************//
			
			//关闭PWM
			EPWM_Stop(EPWM_CH_0_MSK | EPWM_CH_1_MSK|
						EPWM_CH_2_MSK | EPWM_CH_3_MSK|
						EPWM_CH_4_MSK | EPWM_CH_5_MSK);
			//关闭ADC
			ADC_Stop();
			
			//关闭PGA0
			PGA_Stop(PGA0x);

			//关闭PGA1
			PGA_Stop(PGA1x);
      
      //关闭PGA2
      PGA_Stop(PGA2x);
			
			//关闭ACMP1
			#if (ACMP_CH == ACMP_CH0)
			
				ACMP_Stop(ACMP0);

			#endif			
			#if (ACMP_CH == ACMP_CH1)
			
				ACMP_Stop(ACMP1);

			#endif
			
			//关闭DAC
			DAC_Stop();
			
      //关闭FG输出
			#if (Config_CCP_PWM == CCP_PWM_Enable)
				CCP_DisableRun(CCP_PWM_MODULE);
				CCP_Stop(CCP_PWM_MODULE);	

			#endif	
			
			//关闭CCP捕获
			#if (Config_CCP_Capture == CCP_Capture_Enable)
				CCP_DisableRun(CCP_CAPTURE_MODULE);
				CCP_Stop(CCP_CAPTURE_MODULE);			
			#endif
			
			//关闭除法时钟
			CGC_PER12PeriphClockCmd(CGC_PER12Periph_DIV,ENABLE);
			
			//关闭Timer
			TMR_Stop(TMR0);
		//-----------------------------------------------------------------------//	
			//设置唤醒引脚
			#if 1
			//p04输入中断
			GPIO_Init(PORT0,PIN4,PULLDOWN_INPUT);
			GPIO_PinAFInConfig(INT3CFG, IO_INCFG_P04_INTP3);    /*设置P04为INPUT03中端口*/

			INTM->EGP0 &= ~(0x01 << 3);			//设置INPUT03 上升沿触发
			INTM->EGP0 |= 0x01 << 3;
			
			NVIC_EnableIRQ(INTP3_IRQn);
			#endif		

		//-----------------------------------------------------------------------//			
		//深度睡眠模式
//			CGC->PMUKEY = 0x0192A;
//			CGC->PMUKEY = 0x3E4F;
//			CGC->PMUCTL = 1;
			SCB->SCR   |= 0x04;
			__WFI();	
		
		//-----------------------------------------------------------------------//
			//唤醒之后
		//-----------------------------------------------------------------------//	
    	DelayTime_ms(2);
			//关中断
			NVIC_DisableIRQ(INTP3_IRQn);
      
      GPIO_Init(PORT0,PIN4,OUTPUT);
		//-----------------------------------------------------------------------//	
			//恢复功能
			//除法（关时钟之后需要重新配置）
			CGC_PER12PeriphClockCmd(CGC_PER12Periph_DIV,ENABLE);
			DIVSQRT_EnableDIVMode();
			DIVSQRT_EnableSingedMode();
				
			//恢复PWM
			EPWM_Start(EPWM_CH_0_MSK | EPWM_CH_1_MSK|
						EPWM_CH_2_MSK | EPWM_CH_3_MSK|
						EPWM_CH_4_MSK | EPWM_CH_5_MSK);
						
			Bridge_Output_Off()	;
			//恢复ADC
			ADC_Start();	
			
			//恢复PGA0
			PGA_Start(PGA0x);
			
      //恢复PGA1
			PGA_Start(PGA1x);
      
      //恢复PGA2
      PGA_Start(PGA2x);

      //恢复DAC
			DAC_Start();	
			
      //恢复ACMP
      #if (ACMP_CH == ACMP_CH0)
    
      ACMP_Start(ACMP0);

      #endif			
      #if (ACMP_CH == ACMP_CH1)
    
      ACMP_Start(ACMP1);

      #endif
			
      //恢复FG输出
			#if (Config_CCP_PWM == CCP_PWM_Enable)
				CCP_EnableRun(CCP_PWM_MODULE);
				CCP_Start(CCP_PWM_MODULE);	
				
				GPIO_Init(PORT2,PIN3,OUTPUT);
				GPIO_PinAFOutConfig(P23CFG, IO_OUTCFG_P23_CCP0A_O);	

			#endif	
			
			//恢复CCP捕获
			#if (Config_CCP_Capture == CCP_Capture_Enable)
				CCP_EnableRun(CCP_CAPTURE_MODULE);
				CCP_Start(CCP_CAPTURE_MODULE);			
			#endif
      
			//timer
			TMR_Start(TMR0);

			MOTOR_STATE = MC_INIT;
			SYSTEM_STATE = SYS_RUN; 
				
			DelayTime_ms(2);
		//-----------------------------------------------------------------------//	
			//恢复引脚功能	
			
		//-----------------------------------------------------------------------//		
			//恢复中断
			Enable_INT();
			
			SysTick->LOAD  = (uint32_t)(MCU_CLK/1000 - 1UL);
			SysTick->VAL   = 0UL;
			SysTick->CTRL  = 0x00000007;			
		}
	}
	else
	{
		Stru_Time.Sleep = 0;
	}
}
/******************************** END OF FILE *******************************/


