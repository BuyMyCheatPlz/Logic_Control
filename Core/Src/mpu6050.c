/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mpu6050.c
  * @brief          : MPU6050 DMP high-level wrapper
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu_port.h"
#include <math.h>

#define DEFAULT_MPU_HZ 100
#define Q30 1073741824.0f

static uint8_t dmp_initialized = 0;

/**
 * @brief Initialize MPU6050 basic functions
 * @return 0 on success, non-zero on failure
 */
int MPU6050_Init(void)
{
	int result;

	result = mpu_init(NULL);
	if (result != 0)
	{
		return -1;
	}

	result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	if (result != 0)
	{
		return -2;
	}

	result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	if (result != 0)
	{
		return -3;
	}

	result = mpu_set_sample_rate(DEFAULT_MPU_HZ);
	if (result != 0)
	{
		return -4;
	}

	result = mpu_set_gyro_fsr(2000);
	if (result != 0)
	{
		return -5;
	}

	result = mpu_set_accel_fsr(2);
	if (result != 0)
	{
		return -6;
	}

	return 0;
}

/**
 * @brief Initialize MPU6050 DMP
 * @return 0 on success, non-zero on failure
 */
int MPU6050_DMP_Init(void)
{
	int result;

	if (MPU6050_Init() != 0)
	{
		return -1;
	}

	result = dmp_load_motion_driver_firmware();
	if (result != 0)
	{
		return -2;
	}

	result = dmp_set_orientation(
		inv_orientation_matrix_to_scalar(
			(signed char[9]){1, 0, 0,
			                 0, 1, 0,
			                 0, 0, 1}));
	if (result != 0)
	{
		return -3;
	}

	result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
	                            DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
	if (result != 0)
	{
		return -4;
	}

	result = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	if (result != 0)
	{
		return -5;
	}

	result = mpu_set_dmp_state(1);
	if (result != 0)
	{
		return -6;
	}

	dmp_initialized = 1;

	return 0;
}

/**
 * @brief Get quaternion and euler angles from DMP
 * @param quat: Pointer to quaternion structure (can be NULL)
 * @param euler: Pointer to euler angles structure (can be NULL)
 * @return 0 on success, non-zero on failure
 */
int MPU6050_DMP_GetData(MPU6050_Quaternion_t *quat, MPU6050_Euler_t *euler)
{
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat_data[4];
	unsigned long timestamp;

	if (!dmp_initialized)
	{
		return -1;
	}

	if (dmp_read_fifo(gyro, accel, quat_data, &timestamp, &sensors, &more) != 0)
	{
		return -2;
	}

	if (sensors & INV_WXYZ_QUAT)
	{
		if (quat != NULL)
		{
			quat->w = quat_data[0] / Q30;
			quat->x = quat_data[1] / Q30;
			quat->y = quat_data[2] / Q30;
			quat->z = quat_data[3] / Q30;
		}

		if (euler != NULL)
		{
			float q0 = quat_data[0] / Q30;
			float q1 = quat_data[1] / Q30;
			float q2 = quat_data[2] / Q30;
			float q3 = quat_data[3] / Q30;

			euler->pitch = asinf(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 57.3f;
			euler->roll = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.3f;
			euler->yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f;
		}
	}

	return 0;
}

/**
 * @brief Get raw gyroscope data
 * @param gyro: Pointer to gyro structure
 * @return 0 on success, non-zero on failure
 */
int MPU6050_GetGyro(MPU6050_Gyro_t *gyro)
{
	short data[3];
	unsigned long timestamp;

	if (mpu_get_gyro_reg(data, &timestamp) != 0)
	{
		return -1;
	}

	gyro->x = data[0];
	gyro->y = data[1];
	gyro->z = data[2];

	return 0;
}

/**
 * @brief Get raw accelerometer data
 * @param accel: Pointer to accel structure
 * @return 0 on success, non-zero on failure
 */
int MPU6050_GetAccel(MPU6050_Accel_t *accel)
{
	short data[3];
	unsigned long timestamp;

	if (mpu_get_accel_reg(data, &timestamp) != 0)
	{
		return -1;
	}

	accel->x = data[0];
	accel->y = data[1];
	accel->z = data[2];

	return 0;
}

/**
 * @brief Get temperature in degrees Celsius
 * @param temp: Pointer to temperature variable
 * @return 0 on success, non-zero on failure
 */
int MPU6050_GetTemperature(float *temp)
{
	long data;
	unsigned long timestamp;

	if (mpu_get_temperature(&data, &timestamp) != 0)
	{
		return -1;
	}

	*temp = (float)data / 65536.0f + 21.0f;

	return 0;
}

