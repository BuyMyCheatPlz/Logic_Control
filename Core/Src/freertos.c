/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "motor.h"
#include "mpu6050.h"

extern UART_HandleTypeDef huart2;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

enum
{
  UART2_CMD_MAX_LEN = 32,
  UART2_CMD_QUEUE_DEPTH = 4,
  UART2_RX_FRAME_SIZE = 64,
};

static uint8_t uart2_rx_frame[UART2_RX_FRAME_SIZE];
static char uart2_rx_line[UART2_CMD_MAX_LEN];
static uint8_t uart2_rx_len;
static char uart2_cmd_queue[UART2_CMD_QUEUE_DEPTH][UART2_CMD_MAX_LEN];
static volatile uint8_t uart2_cmd_head;
static volatile uint8_t uart2_cmd_tail;
static volatile uint8_t uart2_cmd_count;

static MPU6050_Euler_t mpu_euler;
static MPU6050_Quaternion_t mpu_quat;
static uint8_t mpu_initialized = 0;
/* 默认启用 MPU 航向修正 */
static uint8_t mpu_correction_enabled = 1;

static float base_speed_percent = 0.0f;

/* 默认运动百分比（用于 LEFT/RIGHT/FORWARD/BACK 命令） */
#define DEFAULT_STRAFE_PERCENT 30.0f
#define DEFAULT_FORWARD_PERCENT 30.0f

/* 当使用 Mecanum_SetMotion 设定分轮速度时置位，主循环在此模式下不覆盖各轮目标 */
static uint8_t mecanum_mode_active = 0;
/* 最近一次分轮目标（RR, LR, RF, LF） */
static float mecanum_last_targets[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

/* 先声明 Mecanum_SetMotion 以便下面的步骤函数调用 */
static void Mecanum_SetMotion(float forward, float strafe, float rotation);

/* 物理参数（用于格移动计算） */
#define GRID_SIZE_M 0.30f
#define WHEEL_DIAM_M 0.06f
#define ENCODER_LINES 13
#define ENCODER_QUADRATURE 4
#define GEAR_RATIO 20

/* 计算并执行按格侧移（阻塞）: steps >=1, dir = -1 左, +1 右 */
static void Mecanum_StepStrafe(int steps, int dir)
{
  if (steps <= 0) return;

  /* 计算所需编码器计数（每轮）: counts = steps * GRID * sqrt(2) / circumference * counts_per_wheel_rev */
  float circumference = (float)M_PI * WHEEL_DIAM_M; /* m */
  float wheel_travel = (float)steps * GRID_SIZE_M * 1.41421356237f; /* 因为45度滚轮，车体横移对应车轮走距离乘以 sqrt(2) */
  float wheel_revs = wheel_travel / circumference;
  float counts_per_wheel_rev = (float)(ENCODER_LINES * ENCODER_QUADRATURE * GEAR_RATIO);
  int32_t target_counts = (int32_t)roundf(wheel_revs * counts_per_wheel_rev);

  if (target_counts <= 0)
  {
    return;
  }

  /* 清空并准备累加编码器增量 */
  int32_t acc[MOTOR_COUNT] = {0,0,0,0};
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    /* 读取一次以更新内部 last_count（并忽略返回值） */
    (void)Motor_GetEncoderDelta((MotorId_t)i);
  }

  /* 发起侧移命令，使用默认百分比速度 */
  Mecanum_SetMotion(0.0f, dir * DEFAULT_STRAFE_PERCENT, 0.0f);

  /* 轮询累加各轮绝对编码器增量，直到平均达到目标 */
  while (1)
  {
    int32_t sum = 0;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      int32_t d = Motor_GetEncoderDelta((MotorId_t)i);
      acc[i] += (d >= 0) ? d : -d;
      sum += acc[i];
    }

    int32_t avg = sum / MOTOR_COUNT;
    if (avg >= target_counts)
    {
      break;
    }

    osDelay(10);
  }

  /* 停止并退出 mecanum 模式 */
  base_speed_percent = 0.0f;
  mecanum_mode_active = 0;
  Motor_SetTargetPercent(MOTOR_RIGHT_REAR, 0.0f);
  Motor_SetTargetPercent(MOTOR_LEFT_REAR, 0.0f);
  Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, 0.0f);
  Motor_SetTargetPercent(MOTOR_LEFT_FRONT, 0.0f);
  Motor_StopAll();
  Motor_ResetPID(MOTOR_RIGHT_REAR);
  Motor_ResetPID(MOTOR_LEFT_REAR);
  Motor_ResetPID(MOTOR_RIGHT_FRONT);
  Motor_ResetPID(MOTOR_LEFT_FRONT);
}

/* 计算并执行按格前进/后退（阻塞）: steps >=1, dir = +1 forward, -1 backward */
static void Mecanum_StepForward(int steps, int dir)
{
  if (steps <= 0) return;

  float circumference = (float)M_PI * WHEEL_DIAM_M; /* m */
  float wheel_travel = (float)steps * GRID_SIZE_M; /* 前进不乘 sqrt(2) */
  float wheel_revs = wheel_travel / circumference;
  float counts_per_wheel_rev = (float)(ENCODER_LINES * ENCODER_QUADRATURE * GEAR_RATIO);
  int32_t target_counts = (int32_t)roundf(wheel_revs * counts_per_wheel_rev);

  if (target_counts <= 0) return;

  int32_t acc[MOTOR_COUNT] = {0,0,0,0};
  for (int i = 0; i < MOTOR_COUNT; i++) (void)Motor_GetEncoderDelta((MotorId_t)i);

  /* 使用前进速度启动 */
  Mecanum_SetMotion(dir * DEFAULT_FORWARD_PERCENT, 0.0f, 0.0f);

  while (1)
  {
    int32_t sum = 0;
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      int32_t d = Motor_GetEncoderDelta((MotorId_t)i);
      acc[i] += (d >= 0) ? d : -d;
      sum += acc[i];
    }
    int32_t avg = sum / MOTOR_COUNT;
    if (avg >= target_counts) break;
    osDelay(10);
  }

  base_speed_percent = 0.0f;
  mecanum_mode_active = 0;
  Motor_SetTargetPercent(MOTOR_RIGHT_REAR, 0.0f);
  Motor_SetTargetPercent(MOTOR_LEFT_REAR, 0.0f);
  Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, 0.0f);
  Motor_SetTargetPercent(MOTOR_LEFT_FRONT, 0.0f);
  Motor_StopAll();
  Motor_ResetPID(MOTOR_RIGHT_REAR);
  Motor_ResetPID(MOTOR_LEFT_REAR);
  Motor_ResetPID(MOTOR_RIGHT_FRONT);
  Motor_ResetPID(MOTOR_LEFT_FRONT);
}

// 航向控制 PID 参数
typedef struct {
  float kp;
  float ki;
  float kd;
  float target_pitch;
  float integral;
  float last_error;
  float output;
  float output_limit;
} HeadingPID_t;

static HeadingPID_t heading_pid = {
  .kp = 0.5f,
  .ki = 0.05f,
  .kd = 0.1f,
  .target_pitch = 0.0f,
  .integral = 0.0f,
  .last_error = 0.0f,
  .output = 0.0f,
  .output_limit = 20.0f  // 最大差速 ±20%
};

#define PITCH_CORRECTION_KP 0.5f

// 函数声明
static float HeadingPID_Update(HeadingPID_t *pid, float current_pitch, float dt);
static void HeadingPID_Reset(HeadingPID_t *pid);
static void Mecanum_SetMotion(float forward, float strafe, float rotation);

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Send_Data */
osThreadId_t Send_DataHandle;
const osThreadAttr_t Send_Data_attributes = {
  .name = "Send_Data",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Proccess_Data */
osThreadId_t Proccess_DataHandle;
const osThreadAttr_t Proccess_Data_attributes = {
  .name = "Proccess_Data",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Give_Data */
osMessageQueueId_t Give_DataHandle;
const osMessageQueueAttr_t Give_Data_attributes = {
  .name = "Give_Data"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void UART2_StartReception(void);
static void UART2_EnqueueCommandFromISR(const char *command);
static uint8_t UART2_DequeueCommand(char *command);
static void UART2_SendText(const char *text);
static void UART2_HandleCommand(const char *command);
static void UART2_ProcessRxByte(uint8_t byte);
static void UART2_FinalizeLine(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Give_Data */
  Give_DataHandle = osMessageQueueNew (16, sizeof(uint16_t), &Give_Data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Send_Data */
  Send_DataHandle = osThreadNew(StartTask02, NULL, &Send_Data_attributes);

  /* creation of Proccess_Data */
  Proccess_DataHandle = osThreadNew(StartTask03, NULL, &Proccess_Data_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Send_Data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  UART2_StartReception();
  /* Infinite loop */
  for(;;)
  {
    char command[UART2_CMD_MAX_LEN];

    while (UART2_DequeueCommand(command) != 0U)
    {
      UART2_HandleCommand(command);
    }

    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Proccess_Data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  uint32_t last_tick = osKernelGetTickCount();

  if (MPU6050_DMP_Init() == 0)
  {
    mpu_initialized = 1;
    /* 如果默认启用航向修正，则以当前俯仰为目标并重置 PID */
    if (mpu_correction_enabled)
    {
      if (MPU6050_DMP_GetData(&mpu_quat, &mpu_euler) == 0)
      {
        heading_pid.target_pitch = mpu_euler.pitch;
      }
      else
      {
        heading_pid.target_pitch = 0.0f;
      }
      HeadingPID_Reset(&heading_pid);
    }
  }

  /* Infinite loop */
  for(;;)
  {
    uint32_t current_tick = osKernelGetTickCount();
    float dt_s = (float)(current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;

    float heading_correction = 0.0f;

    // 航向控制循环（独立于电机 PID）
    if (mpu_initialized && mpu_correction_enabled && (base_speed_percent != 0.0f || mecanum_mode_active))
    {
      if (MPU6050_DMP_GetData(&mpu_quat, &mpu_euler) == 0)
      {
        // 使用独立的航向 PID 计算差速
        heading_correction = HeadingPID_Update(&heading_pid, mpu_euler.pitch, dt_s);
      }
    }

    // 应用基础速度 + 航向校正差速
    if (!mecanum_mode_active && (base_speed_percent != 0.0f))
    {
      Motor_SetTargetPercent(MOTOR_RIGHT_REAR, base_speed_percent + heading_correction);
      Motor_SetTargetPercent(MOTOR_LEFT_REAR, base_speed_percent - heading_correction);
      Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, base_speed_percent + heading_correction);
      Motor_SetTargetPercent(MOTOR_LEFT_FRONT, base_speed_percent - heading_correction);
    }
    else if (mecanum_mode_active)
    {
      /* 在 mecanum 分轮模式下，基于记录的分轮目标应用航向校正 */
      Motor_SetTargetPercent(MOTOR_RIGHT_REAR,  mecanum_last_targets[MOTOR_RIGHT_REAR]  + heading_correction);
      Motor_SetTargetPercent(MOTOR_LEFT_REAR,   mecanum_last_targets[MOTOR_LEFT_REAR]   - heading_correction);
      Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, mecanum_last_targets[MOTOR_RIGHT_FRONT] + heading_correction);
      Motor_SetTargetPercent(MOTOR_LEFT_FRONT,  mecanum_last_targets[MOTOR_LEFT_FRONT]  - heading_correction);
    }

    // 电机 PID 控制循环
    Motor_UpdateControl(dt_s);

    osDelay(10);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief 麦克纳姆轮运动控制
 * @param forward: 前进速度 (-100 到 100, 正值前进，负值后退)
 * @param strafe: 平移速度 (-100 到 100, 正值右移，负值左移)
 * @param rotation: 旋转速度 (-100 到 100, 正值顺时针，负值逆时针)
 *
 * 麦克纳姆轮运动学：
 * 右后轮 = forward - strafe - rotation
 * 左后轮 = forward + strafe + rotation
 * 右前轮 = forward + strafe - rotation
 * 左前轮 = forward - strafe + rotation
 */
static void Mecanum_SetMotion(float forward, float strafe, float rotation)
{
  // 计算各轮速度
  float rr = forward - strafe - rotation;  // 右后
  float lr = forward + strafe + rotation;  // 左后
  float rf = forward + strafe - rotation;  // 右前
  float lf = forward - strafe + rotation;  // 左前

  // 找到最大绝对值
  float max_speed = 0.0f;
  if (fabsf(rr) > max_speed) max_speed = fabsf(rr);
  if (fabsf(lr) > max_speed) max_speed = fabsf(lr);
  if (fabsf(rf) > max_speed) max_speed = fabsf(rf);
  if (fabsf(lf) > max_speed) max_speed = fabsf(lf);

  // 归一化到 ±100%
  if (max_speed > 100.0f)
  {
    float scale = 100.0f / max_speed;
    rr *= scale;
    lr *= scale;
    rf *= scale;
    lf *= scale;
  }

  // 设置电机目标速度
  base_speed_percent = (fabsf(forward) + fabsf(strafe) + fabsf(rotation)) / 3.0f;

  HeadingPID_Reset(&heading_pid);
  Motor_ResetPID(MOTOR_RIGHT_REAR);
  Motor_ResetPID(MOTOR_LEFT_REAR);
  Motor_ResetPID(MOTOR_RIGHT_FRONT);
  Motor_ResetPID(MOTOR_LEFT_FRONT);

  Motor_SetTargetPercent(MOTOR_RIGHT_REAR, rr);
  Motor_SetTargetPercent(MOTOR_LEFT_REAR, lr);
  Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, rf);
  Motor_SetTargetPercent(MOTOR_LEFT_FRONT, lf);

  /* 记录分轮目标并进入 mecanum 模式，主循环将不再用统一 base_speed 覆盖 */
  mecanum_last_targets[MOTOR_RIGHT_REAR]  = rr;
  mecanum_last_targets[MOTOR_LEFT_REAR]   = lr;
  mecanum_last_targets[MOTOR_RIGHT_FRONT] = rf;
  mecanum_last_targets[MOTOR_LEFT_FRONT]  = lf;
  mecanum_mode_active = 1;
}

static float HeadingPID_Update(HeadingPID_t *pid, float current_pitch, float dt)
{
  float error = pid->target_pitch - current_pitch;

  // 积分项（带抗饱和）
  pid->integral += error * dt;
  if (pid->integral > pid->output_limit / pid->ki)
  {
    pid->integral = pid->output_limit / pid->ki;
  }
  else if (pid->integral < -pid->output_limit / pid->ki)
  {
    pid->integral = -pid->output_limit / pid->ki;
  }

  // 微分项
  float derivative = (error - pid->last_error) / dt;

  // PID 输出
  float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

  // 输出限幅
  if (output > pid->output_limit)
  {
    output = pid->output_limit;
  }
  else if (output < -pid->output_limit)
  {
    output = -pid->output_limit;
  }

  pid->output = output;
  pid->last_error = error;

  return output;
}

static void HeadingPID_Reset(HeadingPID_t *pid)
{
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
  pid->output = 0.0f;
}

static void UART2_StartReception(void)
{
  uart2_rx_len = 0;
  uart2_cmd_head = 0;
  uart2_cmd_tail = 0;
  uart2_cmd_count = 0;

  if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_rx_frame, UART2_RX_FRAME_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
}

static void UART2_SendText(const char *text)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}

static void UART2_EnqueueCommandFromISR(const char *command)
{
  UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();

  if (uart2_cmd_count < UART2_CMD_QUEUE_DEPTH)
  {
    strncpy(uart2_cmd_queue[uart2_cmd_tail], command, UART2_CMD_MAX_LEN - 1U);
    uart2_cmd_queue[uart2_cmd_tail][UART2_CMD_MAX_LEN - 1U] = '\0';
    uart2_cmd_tail = (uint8_t)((uart2_cmd_tail + 1U) % UART2_CMD_QUEUE_DEPTH);
    uart2_cmd_count++;
  }

  taskEXIT_CRITICAL_FROM_ISR(saved);
}

static uint8_t UART2_DequeueCommand(char *command)
{
  uint8_t has_command = 0U;

  taskENTER_CRITICAL();

  if (uart2_cmd_count > 0U)
  {
    strncpy(command, uart2_cmd_queue[uart2_cmd_head], UART2_CMD_MAX_LEN - 1U);
    command[UART2_CMD_MAX_LEN - 1U] = '\0';
    uart2_cmd_head = (uint8_t)((uart2_cmd_head + 1U) % UART2_CMD_QUEUE_DEPTH);
    uart2_cmd_count--;
    has_command = 1U;
  }

  taskEXIT_CRITICAL();

  return has_command;
}

static void UART2_ProcessRxByte(uint8_t byte)
{
  if ((byte == '\r') || (byte == '\n'))
  {
    UART2_FinalizeLine();
    return;
  }

  if (uart2_rx_len < (UART2_CMD_MAX_LEN - 1U))
  {
    uart2_rx_line[uart2_rx_len++] = (char)byte;
  }
  else
  {
    uart2_rx_len = 0U;
  }
}

static void UART2_FinalizeLine(void)
{
  if (uart2_rx_len > 0U)
  {
    uart2_rx_line[uart2_rx_len] = '\0';
    UART2_EnqueueCommandFromISR(uart2_rx_line);
    uart2_rx_len = 0U;
  }
}

static void UART2_HandleCommand(const char *command)
{
  const char *cursor = command;
  char *end_ptr = NULL;
  long percent;
  char response[64];

  while ((*cursor == ' ') || (*cursor == '\t')) cursor++;

  /* MPU ON/OFF */
  if (strncmp(cursor, "MPU", 3) == 0)
  {
    cursor += 3;
    while ((*cursor == ' ') || (*cursor == '\t')) cursor++;
    if (strncmp(cursor, "ON", 2) == 0)
    {
      if (MPU6050_DMP_GetData(&mpu_quat, &mpu_euler) == 0)
      {
        heading_pid.target_pitch = mpu_euler.pitch;
      }
      else
      {
        heading_pid.target_pitch = 0.0f;
      }
      HeadingPID_Reset(&heading_pid);
      mpu_correction_enabled = 1;
      UART2_SendText("MPU ON\r\n");
      return;
    }
    else if (strncmp(cursor, "OFF", 3) == 0)
    {
      mpu_correction_enabled = 0;
      HeadingPID_Reset(&heading_pid);
      UART2_SendText("MPU OFF\r\n");
      return;
    }
    else
    {
      UART2_SendText("ERR PARAM\r\n");
      return;
    }
  }

  /* PID tuning: PID <kp> <ki> <kd> */
  if (strncmp(cursor, "PID", 3) == 0)
  {
    cursor += 3;
    while ((*cursor == ' ') || (*cursor == '\t')) cursor++;
    char *endp = NULL;
    float kp = strtof(cursor, &endp);
    if (endp == cursor) { UART2_SendText("ERR FORMAT\r\n"); return; }
    cursor = endp; while ((*cursor == ' ') || (*cursor == '\t')) cursor++;
    float ki = strtof(cursor, &endp);
    if (endp == cursor) { UART2_SendText("ERR FORMAT\r\n"); return; }
    cursor = endp; while ((*cursor == ' ') || (*cursor == '\t')) cursor++;
    float kd = strtof(cursor, &endp);
    if (endp == cursor) { UART2_SendText("ERR FORMAT\r\n"); return; }
    while ((*endp == ' ') || (*endp == '\t')) endp++;
    if (*endp != '\0') { UART2_SendText("ERR FORMAT\r\n"); return; }

    heading_pid.kp = kp;
    heading_pid.ki = ki;
    heading_pid.kd = kd;
    HeadingPID_Reset(&heading_pid);
    (void)snprintf(response, sizeof(response), "PID OK %.2f %.2f %.2f\r\n", kp, ki, kd);
    UART2_SendText(response);
    return;
  }

  /* INV command removed */

  /* LEFT <steps> */
  if (strncmp(cursor, "LEFT", 4) == 0)
  {
    cursor += 4; while ((*cursor == ' ') || (*cursor == '\t')) cursor++;
    int steps = 1;
    if (*cursor != '\0')
    {
      char *endp = NULL; long v = strtol(cursor, &endp, 10);
      if ((endp == cursor) || (v <= 0)) { UART2_SendText("ERR FORMAT\r\n"); return; }
      steps = (int)v; while ((*endp == ' ') || (*endp == '\t')) endp++; if (*endp != '\0') { UART2_SendText("ERR FORMAT\r\n"); return; }
    }
    Mecanum_StepStrafe(steps, +1);
    UART2_SendText("LEFT OK\r\n");
    UART2_SendText("STOPPED\r\n");
    return;
  }

  /* RIGHT <steps> */
  if (strncmp(cursor, "RIGHT", 5) == 0)
  {
    cursor += 5; while ((*cursor == ' ') || (*cursor == '\t')) cursor++;
    int steps_r = 1;
    if (*cursor != '\0')
    {
      char *endp2 = NULL; long vr = strtol(cursor, &endp2, 10);
      if ((endp2 == cursor) || (vr <= 0)) { UART2_SendText("ERR FORMAT\r\n"); return; }
      steps_r = (int)vr; while ((*endp2 == ' ') || (*endp2 == '\t')) endp2++; if (*endp2 != '\0') { UART2_SendText("ERR FORMAT\r\n"); return; }
    }
    Mecanum_StepStrafe(steps_r, -1);
    UART2_SendText("RIGHT OK\r\n");
    UART2_SendText("STOPPED\r\n");
    return;
  }

  /* FORWARD <steps> */
  if (strncmp(cursor, "FORWARD", 7) == 0)
  {
    cursor += 7; while ((*cursor == ' ') || (*cursor == '\t')) cursor++;
    int fsteps = 1;
    if (*cursor != '\0')
    {
      char *endp3 = NULL; long vf = strtol(cursor, &endp3, 10);
      if ((endp3 == cursor) || (vf <= 0)) { UART2_SendText("ERR FORMAT\r\n"); return; }
      fsteps = (int)vf; while ((*endp3 == ' ') || (*endp3 == '\t')) endp3++; if (*endp3 != '\0') { UART2_SendText("ERR FORMAT\r\n"); return; }
    }
    Mecanum_StepForward(fsteps, +1);
    UART2_SendText("FORWARD OK\r\n");
    UART2_SendText("STOPPED\r\n");
    return;
  }

  /* BACKWARD <steps>  (also accept BACK) */
  if ((strncmp(cursor, "BACKWARD", 8) == 0) || (strncmp(cursor, "BACK", 4) == 0))
  {
    int offset = (strncmp(cursor, "BACKWARD", 8) == 0) ? 8 : 4;
    cursor += offset; while ((*cursor == ' ') || (*cursor == '\t')) cursor++;
    int bsteps = 1;
    if (*cursor != '\0')
    {
      char *endp4 = NULL; long vb = strtol(cursor, &endp4, 10);
      if ((endp4 == cursor) || (vb <= 0)) { UART2_SendText("ERR FORMAT\r\n"); return; }
      bsteps = (int)vb; while ((*endp4 == ' ') || (*endp4 == '\t')) endp4++; if (*endp4 != '\0') { UART2_SendText("ERR FORMAT\r\n"); return; }
    }
    Mecanum_StepForward(bsteps, -1);
    UART2_SendText("BACK OK\r\n");
    UART2_SendText("STOPPED\r\n");
    return;
  }

  /* STOP */
  if (strncmp(cursor, "STOP", 4) == 0)
  {
    cursor += 4; while ((*cursor == ' ') || (*cursor == '\t')) cursor++;
    if (*cursor != '\0') { UART2_SendText("ERR FORMAT\r\n"); return; }
    base_speed_percent = 0.0f;
    /* 退出 mecanum 分轮模式，清空记录目标 */
    mecanum_mode_active = 0;
    mecanum_last_targets[MOTOR_RIGHT_REAR] = 0.0f;
    mecanum_last_targets[MOTOR_LEFT_REAR] = 0.0f;
    mecanum_last_targets[MOTOR_RIGHT_FRONT] = 0.0f;
    mecanum_last_targets[MOTOR_LEFT_FRONT] = 0.0f;
    HeadingPID_Reset(&heading_pid);
    Motor_SetTargetPercent(MOTOR_RIGHT_REAR, 0.0f);
    Motor_SetTargetPercent(MOTOR_LEFT_REAR, 0.0f);
    Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, 0.0f);
    Motor_SetTargetPercent(MOTOR_LEFT_FRONT, 0.0f);
    Motor_StopAll();
    Motor_ResetPID(MOTOR_RIGHT_REAR); Motor_ResetPID(MOTOR_LEFT_REAR);
    Motor_ResetPID(MOTOR_RIGHT_FRONT); Motor_ResetPID(MOTOR_LEFT_FRONT);
    UART2_SendText("STOP OK\r\n"); return;
  }

  /* RUN <percent> */
  if (strncmp(cursor, "RUN", 3) != 0)
  {
    UART2_SendText("ERR CMD\r\n"); return;
  }

  cursor += 3; while ((*cursor == ' ') || (*cursor == '\t')) cursor++;
  if (*cursor == '\0') { UART2_SendText("ERR PARAM\r\n"); return; }
  percent = strtol(cursor, &end_ptr, 10);
  if ((end_ptr == cursor) || (percent < -100L) || (percent > 100L)) { UART2_SendText("ERR RANGE\r\n"); return; }
  while ((*end_ptr == ' ') || (*end_ptr == '\t')) end_ptr++;
  if (*end_ptr != '\0') { UART2_SendText("ERR FORMAT\r\n"); return; }

  if (percent == 0)
  {
    base_speed_percent = 0.0f; HeadingPID_Reset(&heading_pid); Motor_StopAll();
    Motor_ResetPID(MOTOR_RIGHT_REAR); Motor_ResetPID(MOTOR_LEFT_REAR);
    Motor_ResetPID(MOTOR_RIGHT_FRONT); Motor_ResetPID(MOTOR_LEFT_FRONT);
  }
  else
  {
    base_speed_percent = (float)percent; HeadingPID_Reset(&heading_pid);
    Motor_ResetPID(MOTOR_RIGHT_REAR); Motor_ResetPID(MOTOR_LEFT_REAR);
    Motor_ResetPID(MOTOR_RIGHT_FRONT); Motor_ResetPID(MOTOR_LEFT_FRONT);
    Motor_SetTargetPercent(MOTOR_RIGHT_REAR, base_speed_percent);
    Motor_SetTargetPercent(MOTOR_LEFT_REAR, base_speed_percent);
    Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, base_speed_percent);
    Motor_SetTargetPercent(MOTOR_LEFT_FRONT, base_speed_percent);
  }

  (void)snprintf(response, sizeof(response), "RUN OK %ld\r\n", percent);
  UART2_SendText(response);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    for (uint16_t index = 0U; index < Size; index++)
    {
      UART2_ProcessRxByte(uart2_rx_frame[index]);
    }

    UART2_FinalizeLine();

    if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_rx_frame, UART2_RX_FRAME_SIZE) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart2_rx_len = 0U;
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_rx_frame, UART2_RX_FRAME_SIZE);
  }
}

/* USER CODE END 4 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

