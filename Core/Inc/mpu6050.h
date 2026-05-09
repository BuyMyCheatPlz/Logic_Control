#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MPU6050_ADDR 0x68

typedef struct
{
	float pitch;
	float roll;
	float yaw;
} MPU6050_Euler_t;

typedef struct
{
	float w;
	float x;
	float y;
	float z;
} MPU6050_Quaternion_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} MPU6050_Gyro_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} MPU6050_Accel_t;

int MPU6050_Init(void);
int MPU6050_DMP_Init(void);
int MPU6050_DMP_GetData(MPU6050_Quaternion_t *quat, MPU6050_Euler_t *euler);
int MPU6050_GetGyro(MPU6050_Gyro_t *gyro);
int MPU6050_GetAccel(MPU6050_Accel_t *accel);
int MPU6050_GetTemperature(float *temp);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
