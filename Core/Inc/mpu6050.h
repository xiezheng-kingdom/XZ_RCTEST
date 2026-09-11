#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void MPU6050_Init(void);
void MPU6050_DataUpdate(void);
float MPU6050_GetAx(void);
float MPU6050_GetAy(void);
float MPU6050_GetAz(void);
float MPU6050_GetT(void);
float MPU6050_GetGx(void);
float MPU6050_GetGy(void);
float MPU6050_GetGz(void);
int   MPU6050_IsOK(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H__ */
