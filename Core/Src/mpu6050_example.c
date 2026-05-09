/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mpu6050_example.c
  * @brief          : MPU6050 DMP usage example
  ******************************************************************************
  * 这个文件展示了如何在FreeRTOS任务中使用MPU6050 DMP
  * 可以将这些代码集成到 freertos.c 中
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mpu6050.h"
#include "cmsis_os.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;

/**
 * @brief MPU6050任务示例
 * @param argument: 未使用
 */
void MPU6050_Task_Example(void *argument)
{
	MPU6050_Quaternion_t quat;
	MPU6050_Euler_t euler;
	MPU6050_Gyro_t gyro;
	MPU6050_Accel_t accel;
	float temp;
	char msg[128];
	int result;

	// 延迟等待系统稳定
	osDelay(100);

	// 初始化MPU6050 DMP
	result = MPU6050_DMP_Init();
	if (result != 0)
	{
		snprintf(msg, sizeof(msg), "MPU6050 DMP Init Failed: %d\r\n", result);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

		// 初始化失败，停止任务
		vTaskDelete(NULL);
		return;
	}

	snprintf(msg, sizeof(msg), "MPU6050 DMP Init Success!\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

	// 主循环
	for (;;)
	{
		// 读取DMP数据（四元数和欧拉角）
		if (MPU6050_DMP_GetData(&quat, &euler) == 0)
		{
			// 输出欧拉角
			snprintf(msg, sizeof(msg),
			         "Pitch: %6.2f, Roll: %6.2f, Yaw: %6.2f\r\n",
			         euler.pitch, euler.roll, euler.yaw);
			HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

			// 如果需要四元数
			// snprintf(msg, sizeof(msg),
			//          "Quat: w=%6.3f, x=%6.3f, y=%6.3f, z=%6.3f\r\n",
			//          quat.w, quat.x, quat.y, quat.z);
			// HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
		}

		// 读取原始陀螺仪数据（可选）
		if (MPU6050_GetGyro(&gyro) == 0)
		{
			// snprintf(msg, sizeof(msg),
			//          "Gyro: X=%6d, Y=%6d, Z=%6d\r\n",
			//          gyro.x, gyro.y, gyro.z);
			// HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
		}

		// 读取原始加速度计数据（可选）
		if (MPU6050_GetAccel(&accel) == 0)
		{
			// snprintf(msg, sizeof(msg),
			//          "Accel: X=%6d, Y=%6d, Z=%6d\r\n",
			//          accel.x, accel.y, accel.z);
			// HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
		}

		// 读取温度（可选）
		if (MPU6050_GetTemperature(&temp) == 0)
		{
			// snprintf(msg, sizeof(msg), "Temp: %.2f C\r\n", temp);
			// HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
		}

		// 延迟10ms（100Hz更新率）
		osDelay(10);
	}
}

/**
 * @brief 如何在freertos.c中添加这个任务
 *
 * 1. 在 MX_FREERTOS_Init() 函数中添加任务定义：
 *
 * osThreadId_t MPU6050TaskHandle;
 * const osThreadAttr_t MPU6050Task_attributes = {
 *   .name = "MPU6050Task",
 *   .stack_size = 512 * 4,
 *   .priority = (osPriority_t) osPriorityNormal,
 * };
 *
 * 2. 在创建任务部分添加：
 *
 * MPU6050TaskHandle = osThreadNew(MPU6050_Task_Example, NULL, &MPU6050Task_attributes);
 *
 * 3. 在文件开头添加包含：
 *
 * #include "mpu6050.h"
 */
