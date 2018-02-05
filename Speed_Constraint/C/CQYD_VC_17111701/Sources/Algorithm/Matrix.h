//-------------------------------------------//
//此头文件包含一些通常的矩阵运算
//-------------------------------------------//
#ifndef _MATRIX_H_
#define _MATRIX_H_

#include "PlatformCon.h"
#include <math.h>
#include "GlobalDefine.h"

typedef struct{
  /*  par1
  */
	void(*Init) (void);
	void(*Add)   (float32_t *, float32_t *, uint8_t, uint8_t, float32_t *);
	void(*Sub)   (float32_t *, float32_t *, uint8_t, uint8_t, float32_t *);
	void(*MulM)  (float32_t *, float32_t *, uint8_t, uint8_t, uint8_t, float32_t *);
	void(*Inver) (float32_t *, uint8_t, uint8_t, float32_t *);
	void(*Opp)   (float32_t *, uint8_t, float32_t *);
	void(*MulD)  (float32_t *, uint8_t, uint8_t, float32_t , float32_t *);
  void(*Diag)  (float32_t *, uint8_t, uint8_t, float32_t *);
  float32_t (*Norm) (float32_t *, uint8_t , uint8_t );
  float32_t (*Norm2) (float32_t *, uint8_t , uint8_t );
  void (*EG)    (float32_t *, uint8_t);
  void(*UD)     (float32_t *, uint8_t, float32_t *, float32_t *);
}MATRIX_Info;

extern MATRIX_Info Mat_Cal_Info;

void Matrix_Init(void);

#endif

