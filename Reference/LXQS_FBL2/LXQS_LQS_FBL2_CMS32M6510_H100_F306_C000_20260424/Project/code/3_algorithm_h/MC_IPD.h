//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    FOC_IPD.h
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   
*---------------------------------------------------------------------------//
*****************************************************************************/

//==========================================================================//

#ifndef __FOC_IPD_H
#define __FOC_IPD_H

/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/

#define		AH												(0x0001ul)  //0000 0001		
#define		BH												(0x0004ul)	//0000 0100			
#define		CH												(0x0010ul)  //0001 0000

#define		AL												(0x0002ul) 	//0000 0010			
#define		BL												(0x0008ul)  //0000 1000		
#define		CL												(0x0020ul)	//0010 0000

#define 	PWM_ON_AH									(0xFEFF)		//1111 1110 1111 1111
#define 	PWM_ON_AL									(0xFDFF)		//1111 1101 1111 1111
#define 	PWM_ON_BH									(0xFBFF)		//1111 1011 1111 1111
#define 	PWM_ON_BL									(0xF7FF)		//1111 0111 1111 1111
#define 	PWM_ON_CH									(0xEFFF)		//1110 1111 1111 1111
#define 	PWM_ON_CL									(0xDFFF)		//1101 1111 1111 1111

/*POSITION CHECK USE*/

#define		PDRIVE_A_B								(0X3F00 |AH | BL)
#define		PDRIVE_A_C								(0X3F00 |AH | CL)
#define		PDRIVE_B_C								(0X3F00 |BH | CL)
#define		PDRIVE_B_A								(0X3F00 |BH | AL)
#define		PDRIVE_C_A								(0X3F00 |CH | AL)
#define		PDRIVE_C_B								(0X3F00 |CH | BL)


/*CHARGE USE*/
#define   DRIVER_ALPWM							(0X3F00 & PWM_ON_AL)					
#define   DRIVER_BLPWM							(0X3F00 & PWM_ON_BL)	
#define   DRIVER_CLPWM							(0X3F00 & PWM_ON_CL)	

/*OTHER USE*/
#define  	DRIVER_OFF								(0X3F00)
#define  	DRIVER_ON								  (0X0000)
#define  	DRIVER_AL									(0X3F00 | AL)
#define  	DRIVER_BL									(0X3F00 | BL)
#define  	DRIVER_CL									(0X3F00 | CL)
#define  	DRIVER_ABCL								(0X3F00 | AL | BL | CL)
#define  	DRIVER_ABCH								(0X3F00 | AH | BH | CH)


/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/
typedef struct
{	   		    
	//---------------------------------------------------------------------------/	
	uint8_t			InjectAngleIndex;		    	// 最值扇区	

  uint32_t			SqTime1;            		// 第一阶段发波时长 us    
  uint32_t			SqTime2;               	// 第二阶段发波时长 us
	
	int16_t				IPD_Cur_AdMin;					
	int16_t				IPD_Cur_AdMax;	
	int16_t				IPD_Cur_AdValue [8];		// IPD电流值
	
  int16_t				ReturnTheta;            // IPD返回角度
	
}Struct_SquareIPD;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/

extern Struct_SquareIPD			Stru_IPD_Pulse 		;

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/
void MC_IPD_Ctrl_Init(void);
void MC_IPD_Ctrl_Recv(void);

uint32_t FOC_IPD_Square_SWMode(Struct_SquareIPD *p);
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/

#endif


/******************************** END OF FILE *******************************/
