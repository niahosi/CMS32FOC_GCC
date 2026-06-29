

//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    pid.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   用于放置PI控制函数
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
Struct_PI								Stru_PI_Id						= {0};					//d轴电流环
Struct_PI								Stru_PI_Iq						= {0};					//q轴电流环

//---------------------------------------------------------------------------/
//	Global variable definitions(declared in header file with 'extern')
//---------------------------------------------------------------------------/

//---------------------------------------------------------------------------/
//	Local function prototypes('static')
//---------------------------------------------------------------------------/


//===========================================================================/
//***** definitions  end ****************************************************/
//===========================================================================/

/**
  * @brief  PIQ30控制器初始化函数,将积分时间关联到积分系数中去
  * @param  hPI     PI控制器句柄
  * @param  kp      比例系数
  * @param  ki      积分系数
  * @param  max     输出上限
  * @param  min     输出下限
  * @param  FixP    内部信号的q格式
  *
  * @retval None
  */
void PIQ30_Init(Struct_PIQ30 *hPI,int32_t kp,int32_t ki,uint8_t Qkp,uint8_t Qki,int32_t max,int32_t min,uint8_t FixP)
{
  hPI->SigFixP = FixP;                       //PI控制器内部信号的Q格式
  hPI->qKp = Qkp;
  hPI->qKi = Qki;
  hPI->Kp = kp;                           //比例系数分子
  hPI->Ki = ki;                           //积分系数分子
	
  hPI->ProportionTerm = 0;               //比例项
  hPI->IntegralTerm = 0;                 //积分项

	hPI->LowerIntegralLimit = min;
	hPI->LowerOutputLimit = min;
	hPI->UpperIntegralLimit = max;
	hPI->UpperOutputLimit = max;

  hPI->Error = 0;                        //误差值
  hPI->Output = 0;                       //PI输出值
}
/*****************************************************************************
* Function Name  : PIQ30_Controller
* Description    : PI计算  
* Function Call  : 
* Input Paragram : PI结构体变量地址、误差值（参考值 - 实际值）
* Return Value   : PI计算结果
* note           : 
* Version        : V0.1    2024/07/05    新建			Wj
******************************************************************************/
int32_t PIQ30_Controller(Struct_PIQ30 *hPI, int32_t error)
{
  uint8_t fixperror = 30 - hPI->SigFixP;

  //误差缩小;
  hPI->Error = error>> fixperror;

  //比例项;
  hPI->ProportionTerm = (int32_t)((int64_t)hPI->Kp * hPI->Error >> hPI->qKp);
  //积分项;
  hPI->IntegralTerm =  hPI->IntegralTerm + (int32_t)(((int64_t)hPI->Ki*hPI->Error)>>hPI->qKi);
  hPI->IntegralTerm =  FIXP_sat(hPI->IntegralTerm,hPI->UpperIntegralLimit,hPI->LowerIntegralLimit);
	
  //输出项;
  hPI->Output = hPI->IntegralTerm + hPI->ProportionTerm;
  //输出限幅
  hPI->Output =  FIXP_sat(hPI->Output,hPI->UpperOutputLimit,hPI->LowerOutputLimit);

  return (hPI->Output<<fixperror);
}




/*****************************************************************************
* Function Name  : PI_Controller
* Description    : PI计算   位置式PI
* Function Call  : 
* Input Paragram : PI结构体变量地址、误差值（参考值 - 实际值）
* Return Value   : PI计算结果
* note           : 
* Version        : V0.1    2024/07/05    新建			Wj
******************************************************************************/
int32_t PI_Controller(Struct_PI *pi, int32_t error)
{
    //积分项计算及边界限制处理
    pi->IntegralTerm = pi->IntegralTerm + pi->Ki * error;
    
    if (pi->IntegralTerm > pi->UpperIntegralLimit)
    {
        pi->IntegralTerm = pi->UpperIntegralLimit;
    }
    if (pi->IntegralTerm < pi->LowerIntegralLimit)
    {
        pi->IntegralTerm = pi->LowerIntegralLimit;
    }

    //输出值计算及边界限制处理
    pi->Output = ( pi->Kp * error >> pi->qKp) + (pi->IntegralTerm >> pi->qKi);

    if (pi->Output > pi->UpperOutputLimit)
    {
        pi->Output = pi->UpperOutputLimit;
    }
    
    if (pi->Output < pi->LowerOutputLimit)
    {
        pi->Output = pi->LowerOutputLimit;
    } 
 
    //记录误差值
    pi->Error = error;  

    return(pi->Output);
}




/*****************************************************************************
* Function Name  : PI_Para_Init
* Description    : PI结构体初始化
* Function Call  : PI_Para_Init
* Input Paragram : *PI  PI控制器句柄,kp比例系数,ki积分系数,max输出上限,min输出下限
* Return Value   : None
* note           : 
* Version        : V0.1    2025/01/06    新建			Xh
******************************************************************************/
void PI_Para_Init(Struct_PI *PI, int32_t kp, int32_t ki, uint8_t Qkp, uint8_t Qki, int16_t max, int16_t min)
{
  PI->qKp = Qkp;                        //比例系数Q格式
  PI->qKi = Qki;												//积分系数Q格式
	
  PI->Kp = kp;                          // 比例系数分子
  PI->Ki = ki;                          // 积分系数分子
	
  PI->IntegralTerm = 0;                 // 积分项

	PI->LowerIntegralLimit = min << Qki;
	PI->LowerOutputLimit = min;
	PI->UpperIntegralLimit = max << Qki;
	PI->UpperOutputLimit = max;

  PI->Error = 0;                        // 误差值
  PI->Output = 0;                       // PI输出值
}

/*****************************************************************************
* Function Name  : PI_Set_Limit
* Description    : PI结构体初始化
* Function Call  : PI_Para_Init
* Input Paragram : *PI  PI控制器句柄,kp比例系数,ki积分系数,max输出上限,min输出下限
* Return Value   : None
* note           : 
* Version        : V0.1    2025/01/06    新建			Xh
******************************************************************************/
void PI_Set_Limit(Struct_PI *PI, int32_t Upper, int32_t Lower)
{
	
	PI->UpperOutputLimit = Upper;
	PI->UpperIntegralLimit = Upper << PI->qKi;
	
	PI->LowerOutputLimit = Lower;
	PI->LowerIntegralLimit = Lower << PI->qKi;
	
}

void PI_Set_Integrater(Struct_PI *PI, int32_t temp)
{
	
	PI->IntegralTerm = temp << PI->qKi;
	
//	 temp*(1<<PI->qKi);需要验证一下看是否有必要
}

/******************************** END OF FILE *******************************/







