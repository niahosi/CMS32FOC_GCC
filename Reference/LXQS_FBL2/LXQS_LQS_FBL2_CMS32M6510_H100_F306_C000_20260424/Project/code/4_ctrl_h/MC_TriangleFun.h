#ifndef __MC_TRIANGLEFUN_H
#define __MC_TRIANGLEFUN_H


#include "stdint.h"


#define TRIANGLEFUN_HIGHPRESICION (0)																		//高精度正余弦查表
#define TRIANGLEFUN_LOWPRESICION  (1)																		//低精度正余弦查表
#define TRIANGLEMODE							(TRIANGLEFUN_HIGHPRESICION)




//正余弦查表掩码
#define SIN_MASK        0x0300u
#define U0_90           0x0200u
#define U90_180         0x0300u
#define U180_270        0x0000u
#define U270_360        0x0100u

	
//高精度正余弦查表
#define SIN_MASK_HIGHPRECISION        0xC000u
#define U0_90_HIGHPRECISION           0x0000u
#define U90_180_HIGHPRECISION         0x4000u
#define U180_270_HIGHPRECISION        0x8000u
#define U270_360_HIGHPRECISION        0xC000u





#endif

