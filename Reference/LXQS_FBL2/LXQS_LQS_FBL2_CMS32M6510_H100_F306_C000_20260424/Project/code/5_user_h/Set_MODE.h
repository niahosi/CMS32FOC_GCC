//===========================================================================//
/*****************************************************************************/
//---------------------------------------------------------------------------//
/** \file
*  File Name  : Set_MODE.h
*  Author     : CMS_Motor_Control_Team
*  Version    : 第二代风机软件平台
*  Date       : 09/01/2022
*  Description: 本页参数为平台模式、功能选择相关参数，
								包括但不限于：停车方式、开环模式、通信方式等等等
**             
//---------------------------------------------------------------------------//
******************************************************************************/

//===========================================================================//
#ifndef __SET_MODE_H
#define __SET_MODE_H


//============================================================================/
// 代码对齐方式：Edit--Configuration--Editor--C/C++Files--Tab size = 2
//----------------------------------------------------------------------------/

//===========================================================================//
//-------------------------------电机旋转方向----------------------------------/
// 电机方向																															
#define		DIR_CW																(0)											// 顺时针旋转   
#define		DIR_CCW																(1)											// 逆时针旋转 
#define		CONFIG_MOTOR_DIR											(DIR_CCW)						  

//-------------------------------休眠功能--------------------------------------/
// 休眠
#define  Sleep_Disable                      		(0)                      // 休眠功能关闭
#define  Sleep_Enable                       		(1)                      // 休眠功能开启
#define  Sleep_Control_Mode                 		(Sleep_Disable)

//-------------------------------通信方式配置----------------------------------/
// 通信设置
#define		Comm_None															(0)                     // 无通信模式
#define		Uart_Scope														(1)                     // 串口上位机数据监测，只发送
#define		Uart_UI																(2)                     // 串口用户界面，使能收、发
#define		Jlink_RTT															(3)											// Jlink RTT查看数据
#define		Config_Comm_Mode											(Jlink_RTT)						

//--------------------------------母线纹波电压补偿-----------------------------/
// 市电整流供电时可用，一定程度上解决电压纹波导致的相电流拍频问题
#define		Vbus_Comp_DISABLE											(0)                     // 母线纹波电压补偿
#define		Vbus_Comp_ENABLE											(1)                     // 母线纹波电压补偿开启
#define		Config_VbusRipple_Comp								(Vbus_Comp_DISABLE)			

//-------------------------------升速功能----------------------------------/
// 
#define		SpeedUp_None													(0)											// 升速功能关闭  
#define		Over_Modulate													(1)											// 过调制升速  
#define		Flux_Weaken														(2)											// 弱磁升速   
#define		OverAndWeaken													(3)											// 弱磁 + 过调制
#define		Config_SpeedUp_Mode										(SpeedUp_None)

#define		WEAKENING_THRESHOLD										(0.96)									// 弱磁阈值
#define   WEAKFLUX_KP														_Q15(0.1)								// 弱磁PI参数：Kp                               
#define   WEAKFLUX_Ki														_Q15(0.001)							// 弱磁PI参数：Ki  
#define   MAX_LEADANGLE													_ANGLE_DEG2PU(60)		    // 最大超前角 单位：°
//-------------------------------预充电模式------------------------------------/
// 双N电路可使能，PN无需使用
#define		CHARGE_DISABLE                     		(0)                     // 预充电关闭
#define		CHARGE_ENABLE                      		(1)                     // 预充电使能
#define		Config_CHARGE_MODE                    (CHARGE_DISABLE)							
#define		CHARGE_TIME                        		(0.1)                  // 预充电时间 单位：s

//-------------------------------内部BG参考校准----------------------------------/
// 使用内部基准校准VCC供电电压，供电LDO精度较差时可使用
#define		BANDGAP_DISABLE												(0)                      // 关闭Bandgap反校功能
#define		BANDGAP_ENABLE												(1)                      // 开启Bandgap反校功能
#define		CONFIG_BANDGAP_MODE										(BANDGAP_DISABLE)

//-------------------------------转子初始位置获取方式选择----------------------------------/
#define	  POSITION_NONE													(0)											 //不获取转子初始位置
#define		POSITION_PULSE_INJECTION							(1)											 //脉冲注入法获取初始位置
#define		POSITION_ALIGN												(2)											 //锁定法获取初始位置
#define   CONFIG_INITIAL_POSITION								(POSITION_NONE) 

//--------------------------------脉冲注入时间------------------------------//
//采用脉冲注入法来获取转子位置时使用
#define   IPD_SQUARE_TIME1                      (50)                     // 第一阶段脉冲时间 单位 us
#define   IPD_SQUARE_TIME2                      (120)                     // 第二阶段脉冲时间 单位 us


//-------------------------------顺逆风参数----------------------------------/
// 顺逆风检测功能  
#define  Start_NoWind														(0)											// 顺风检测功能关闭
#define  Start_Wind															(1)											// 顺风检测功能开启
#define  Config_Wind_Mode												(Start_NoWind)

//顺逆风检测模式选择
#define  WINDCHECK_MODE1											   (0)										//顺逆风检测模式1   零电压法	
#define  WINDCHECK_MODE2												 (1)										//顺逆风检测模式2   零电流法 
#define  CONFIG_WINDCHECK_MODE									 (WINDCHECK_MODE2)			

#define  WIND_KP																 _Q15(0.95)							//锁相环比例系数
#define  LOWSPEED_THRESHOLD											 (70)										//单位    RPM
//下面两个参数用于判定电机是否处于静止状态    判定方法：在时间 STILL_JUDGE_TIME 内，转子的位置波动不超过 STILL_JUDGE_THETAERROR 时，则认为电机静止
#define  STILL_JUDGE_TIME												 (0.15)								  //单位    s
#define  STILL_JUDGE_THETAERROR  								 (4.5)									//单位    °




//-------------------------------SVPWM调制方式----------------------------------/
// 五段式说明：双电阻直接使用 SEGMENT_5，将减少1/3开关损耗；
//						 单电阻直接使用 SEGMENT_5 在低负载情况下电流失真严重，建议使用SEGMENT_5_Lim，开关损耗减少量小于1/3
#define		SEGMENT_5															(0)											// 五段式发波
#define		SEGMENT_5_Lim													(1)											// 最小矢量保护五段式(适用于单电阻)
#define		SEGMENT_7															(2)											// 七段式发波
#define		CONFIG_SVPWM_MODE											(SEGMENT_7)							// 发波方式   

//-------------------------------死区补偿模式设置----------------------------------/
// 需根据硬件条件调整补偿系数、补偿角度
#define		DeadComp_Disable											(0)											// 死区补偿关闭
#define		DeadComp_Enable												(1)											// 死区补偿使能
#define		CONFIG_DeadComp_MODE									(DeadComp_Disable)						 

//-------------------------------电机停车模式设置-------------------------------/
#define		MOTOR_FREE_STOP                 		  (0)                     // 自由停车
#define		MOTOR_BRAKE_STOP										  (1)                     // 刹车
#define		MOTOR_STOP_MODE												(BRAKE_STOP_DISABLE)

#define		TIME_STOP_HOLD                     		(2000)                  // 停机延时					超时直接关闭MOS		单位：ms
#define		BRAKE_STOP_SPEED                   		(2000)                  // 刹车转速 				低于此转速刹车		单位：RPM
#define		BRAKE_STOP_TIME                    		(3500)                  // 刹车时间 													单位：ms
#define		MOTOR_SPEED_STOP                   		(100)                   // 电机停机判段转速 低于此转速MOS全关 单位：RPM 
//-------------------------------电机测试模式设置----------------------------------/
#define		HWTEST_OFF                						(0)											// 测试模式关闭
#define		HWTEST_PWM                 						(1)											// 固定占空比测试 UVW：20%、30%、40%
#define		HWTEST_VOLT_DRAG											(2)											// 开环电压拖动测试
#define		HWTEST_CURR_DRAG											(3)											// 开环电流拖动测试
#define		HWTEST_Hall_Learn											(4)											
#define		HWTEST_Debug													(5)								
#define		Config_HWTEST_MODE                 		(HWTEST_Debug)

// 开环拖动参数
#define		Openloop_Curr_Max											(0.5)										// 拖动相电流峰值  A
#define		Openloop_Volt_Max											(3000)                  // 拖动电压最大值  0-32767

#define		Openloop_Speed_Max										(100)                   // 最高拖动转速   r/min
#define		Openloop_AccTime											(10)                    // 开环拖动加速过程时长 单位：s 

#endif

/******************************** END OF FILE *******************************/

