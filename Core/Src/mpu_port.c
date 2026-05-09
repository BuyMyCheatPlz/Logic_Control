/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mpu_port.c
  * @brief          : MPU6050 DMP I2C port layer for STM32 HAL
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mpu_port.h"
#include "i2c.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;

#define MPU_I2C_TIMEOUT 100

/**
 * @brief Write data to MPU6050 via I2C
 * @param slave_addr: 7-bit I2C slave address (will be shifted left by HAL)
 * @param reg_addr: Register address to write to
 * @param length: Number of bytes to write
 * @param data: Pointer to data buffer
 * @return 0 on success, non-zero on failure
 */
int i2c_write(unsigned char slave_addr, unsigned char reg_addr,
              unsigned char length, unsigned char const *data)
{
	HAL_StatusTypeDef status;
	uint8_t buffer[256];

	if (length > 255)
	{
		return -1;
	}

	buffer[0] = reg_addr;
	for (uint8_t i = 0; i < length; i++)
	{
		buffer[i + 1] = data[i];
	}

	status = HAL_I2C_Master_Transmit(&hi2c1, slave_addr << 1, buffer, length + 1, MPU_I2C_TIMEOUT);

	return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief Read data from MPU6050 via I2C
 * @param slave_addr: 7-bit I2C slave address (will be shifted left by HAL)
 * @param reg_addr: Register address to read from
 * @param length: Number of bytes to read
 * @param data: Pointer to data buffer
 * @return 0 on success, non-zero on failure
 */
int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
             unsigned char length, unsigned char *data)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c1, slave_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, length, MPU_I2C_TIMEOUT);

	return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief Delay in milliseconds
 * @param num_ms: Number of milliseconds to delay
 */
void delay_ms(unsigned long num_ms)
{
	if (osKernelGetState() == osKernelRunning)
	{
		osDelay(num_ms);
	}
	else
	{
		HAL_Delay(num_ms);
	}
}

/**
 * @brief Get current system time in milliseconds
 * @param count: Pointer to store the current time
 */
void get_ms(unsigned long *count)
{
	if (osKernelGetState() == osKernelRunning)
	{
		*count = osKernelGetTickCount();
	}
	else
	{
		*count = HAL_GetTick();
	}
}

/**
 * @brief No operation (NOP) instruction
 */
void __no_operation(void)
{
	__NOP();
}

/**
 * @brief Minimum of two integers
 */
int min(int a, int b)
{
	return (a < b) ? a : b;
}

/**
 * @brief Register interrupt callback (stub)
 */
void reg_int_cb(struct int_param_s *param)
{
	(void)param;
}

/**
 * @brief Convert orientation matrix to scalar
 */
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
	unsigned short scalar;
	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;
	return scalar;
}

/**
 * @brief Convert row to scale
 */
unsigned short inv_row_2_scale(const signed char *row)
{
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;
	return b;
}
