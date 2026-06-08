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

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
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
  UART2_CMD_QUEUE_DEPTH = 16,
  UART2_RX_FRAME_SIZE = 64,

  UART3_CMD_MAX_LEN = 32,
  UART3_RX_FRAME_SIZE = 32,
};

static uint8_t uart2_rx_frame[UART2_RX_FRAME_SIZE];
static char uart2_rx_line[UART2_CMD_MAX_LEN];
static uint8_t uart2_rx_len;
static char uart2_cmd_queue[UART2_CMD_QUEUE_DEPTH][UART2_CMD_MAX_LEN];
static volatile uint8_t uart2_cmd_head;
static volatile uint8_t uart2_cmd_tail;
static volatile uint8_t uart2_cmd_count;

static uint8_t uart3_rx_frame[UART3_RX_FRAME_SIZE];
static char uart3_rx_line[UART3_CMD_MAX_LEN];
static uint8_t uart3_rx_len;

/* UART2 数组模式状态机 */
static uint8_t uart2_in_array;

static float base_speed_percent = 0.0f;

/* UART3 发送函数前置声明 */
static void UART3_SendText(const char *text);

/* 当使用 Mecanum_SetMotion 设定分轮速度时置位，主循环在此模式下不覆盖各轮目标 */
static uint8_t mecanum_mode_active = 0;
/* 最近一次分轮目标（RR, LR, RF, LF） */
static float mecanum_last_targets[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float heading_pitch_lpf = 0.0f;
static uint8_t heading_pitch_lpf_initialized = 0;

typedef enum
{
  MOTION_KIND_STRAFE = 0,
  MOTION_KIND_FORWARD = 1,
  MOTION_KIND_CIRCLE = 2,
} MotionKind_t;

typedef struct
{
  MotionKind_t kind;
  int steps;
  int dir;
  uint8_t source;   /* 2 = UART2, 3 = UART3 */
} MotionRequest_t;

enum
{
  MOTION_QUEUE_DEPTH = 16,
};

static MotionRequest_t motion_queue[MOTION_QUEUE_DEPTH];
static volatile uint8_t motion_queue_head;
static volatile uint8_t motion_queue_tail;
static volatile uint8_t motion_queue_count;
static uint8_t motion_active;
static MotionRequest_t motion_current;
static uint32_t motion_start_tick;

/* 先声明 Mecanum_SetMotion 以便下面的步骤函数调用 */
static void Mecanum_SetMotion(float forward, float strafe, float rotation) __attribute__((unused));

/* 物理参数已集中到 config.h */
static void UART2_SendText(const char *text);

#include "config.h"

static uint8_t MotionQueue_Enqueue(const MotionRequest_t *request);
static void MotionQueue_Clear(void);
static uint8_t Motion_StartRequest(const MotionRequest_t *request);
static uint8_t Motion_Tick(void);

static uint32_t Motion_ComputeTargetCounts(MotionKind_t kind, int steps)
{
  if (steps <= 0)
  {
    return 0U;
  }

  float circumference = (float)M_PI * WHEEL_DIAM_M;

  float wheel_travel = 0.0f;

  if (kind == MOTION_KIND_STRAFE)
  {
    wheel_travel = (float)steps * GRID_SIZE_M * STRAFE_CORRECTION_FACTOR;
  }
  else if (kind == MOTION_KIND_CIRCLE)
  {
    wheel_travel = (float)steps * CIRCLE_TURN_WHEEL_TRAVEL_M * TURN_CORRECTION_FACTOR;
  }
  else
  {
    wheel_travel = (float)steps * GRID_SIZE_M * FORWARD_CORRECTION_FACTOR;
  }

  float wheel_revs = wheel_travel / circumference;
  float counts_per_wheel_rev = (float)(ENCODER_LINES * ENCODER_QUADRATURE * GEAR_RATIO);
  return (uint32_t)roundf(wheel_revs * counts_per_wheel_rev);
}

static uint8_t MotionQueue_Enqueue(const MotionRequest_t *request)
{
  if (request == NULL)
  {
    return 0U;
  }

  if (motion_queue_count >= MOTION_QUEUE_DEPTH)
  {
    return 0U;
  }

  motion_queue[motion_queue_tail] = *request;
  motion_queue_tail = (uint8_t)((motion_queue_tail + 1U) % MOTION_QUEUE_DEPTH);
  motion_queue_count++;
  return 1U;
}

static void MotionQueue_Clear(void)
{
  motion_queue_head = 0U;
  motion_queue_tail = 0U;
  motion_queue_count = 0U;
}

static uint8_t Motion_StartRequest(const MotionRequest_t *request)
{
  if ((request == NULL) || motion_active)
  {
    return 0U;
  }

  uint32_t target_counts = Motion_ComputeTargetCounts(request->kind, request->steps);
  if (target_counts == 0U)
  {
    return 0U;
  }

  motion_current = *request;
  motion_start_tick = osKernelGetTickCount();

  base_speed_percent = 0.0f;
  mecanum_mode_active = 0;
  Motor_SetTargetPercent(MOTOR_RIGHT_REAR, 0.0f);
  Motor_SetTargetPercent(MOTOR_LEFT_REAR, 0.0f);
  Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, 0.0f);
  Motor_SetTargetPercent(MOTOR_LEFT_FRONT, 0.0f);
  Motor_ResetPID(MOTOR_RIGHT_REAR);
  Motor_ResetPID(MOTOR_LEFT_REAR);
  Motor_ResetPID(MOTOR_RIGHT_FRONT);
  Motor_ResetPID(MOTOR_LEFT_FRONT);

  float pwm_limit = (COMMAND_MAX_OUTPUT_PERCENT / 100.0f) * MOTOR_PWM_PERIOD;
  Motor_SetOutputLimit(MOTOR_RIGHT_REAR,  -pwm_limit, pwm_limit);
  Motor_SetOutputLimit(MOTOR_LEFT_REAR,   -pwm_limit, pwm_limit);
  Motor_SetOutputLimit(MOTOR_RIGHT_FRONT, -pwm_limit, pwm_limit);
  Motor_SetOutputLimit(MOTOR_LEFT_FRONT,  -pwm_limit, pwm_limit);

  /* Restore per-wheel position output limits to their normal operation values.
   * Startup limits (POSITION_OUTPUT_LIMIT_STARTUP_xx) are only for power-on; once
   * a real position command starts, switch to the full limit. */
  Motor_SetPositionOutputLimit(MOTOR_RIGHT_REAR,  POSITION_OUTPUT_LIMIT_RR);
  Motor_SetPositionOutputLimit(MOTOR_LEFT_REAR,   POSITION_OUTPUT_LIMIT_RL);
  Motor_SetPositionOutputLimit(MOTOR_RIGHT_FRONT, POSITION_OUTPUT_LIMIT_FR);
  Motor_SetPositionOutputLimit(MOTOR_LEFT_FRONT,  POSITION_OUTPUT_LIMIT_FL);

  if (request->kind == MOTION_KIND_STRAFE)
  {
    float rr = 0.0f - (float)request->dir * DEFAULT_STRAFE_PERCENT;
    float lr = 0.0f + (float)request->dir * DEFAULT_STRAFE_PERCENT;
    float rf = 0.0f + (float)request->dir * DEFAULT_STRAFE_PERCENT;
    float lf = 0.0f - (float)request->dir * DEFAULT_STRAFE_PERCENT;

    Motor_SetPositionTarget(MOTOR_RIGHT_REAR,  (rr >= 0.0f) ? (int32_t)target_counts : -(int32_t)target_counts);
    Motor_SetPositionTarget(MOTOR_LEFT_REAR,   (lr >= 0.0f) ? (int32_t)target_counts : -(int32_t)target_counts);
    Motor_SetPositionTarget(MOTOR_RIGHT_FRONT, (rf >= 0.0f) ? (int32_t)target_counts : -(int32_t)target_counts);
    Motor_SetPositionTarget(MOTOR_LEFT_FRONT,  (lf >= 0.0f) ? (int32_t)target_counts : -(int32_t)target_counts);
  }
  else if (request->kind == MOTION_KIND_CIRCLE)
  {
    int32_t signed_counts = (request->dir >= 0) ? (int32_t)target_counts : -(int32_t)target_counts;

    Motor_SetPositionTarget(MOTOR_RIGHT_REAR,  -signed_counts);
    Motor_SetPositionTarget(MOTOR_LEFT_REAR,    signed_counts);
    Motor_SetPositionTarget(MOTOR_RIGHT_FRONT, -signed_counts);
    Motor_SetPositionTarget(MOTOR_LEFT_FRONT,   signed_counts);
  }
  else
  {
    int32_t signed_counts = (request->dir >= 0) ? (int32_t)target_counts : -(int32_t)target_counts;
    Motor_SetPositionTarget(MOTOR_RIGHT_REAR,  signed_counts);
    Motor_SetPositionTarget(MOTOR_LEFT_REAR,   signed_counts);
    Motor_SetPositionTarget(MOTOR_RIGHT_FRONT, signed_counts);
    Motor_SetPositionTarget(MOTOR_LEFT_FRONT,  signed_counts);
  }

  motion_active = 1U;
  return 1U;
}

static uint8_t Motion_Tick(void)
{
  if (!motion_active)
  {
    if (motion_queue_count == 0U)
    {
      return 0U;
    }

    MotionRequest_t request = motion_queue[motion_queue_head];
    motion_queue_head = (uint8_t)((motion_queue_head + 1U) % MOTION_QUEUE_DEPTH);
    motion_queue_count--;
    return Motion_StartRequest(&request);
  }

  uint8_t all_reached = 1U;
  for (MotorId_t motor = MOTOR_RIGHT_REAR; motor < MOTOR_COUNT; motor++)
  {
    if (!Motor_PositionReached(motor, POSITION_TOLERANCE_COUNTS))
    {
      all_reached = 0U;
      break;
    }
  }

  if (!all_reached && ((osKernelGetTickCount() - motion_start_tick) <= POSITION_TIMEOUT_MS))
  {
    return 0U;
  }

  Motor_ClearPositionTarget(MOTOR_RIGHT_REAR);
  Motor_ClearPositionTarget(MOTOR_LEFT_REAR);
  Motor_ClearPositionTarget(MOTOR_RIGHT_FRONT);
  Motor_ClearPositionTarget(MOTOR_LEFT_FRONT);
  Motor_StopAll();
  Motor_ResetPID(MOTOR_RIGHT_REAR);
  Motor_ResetPID(MOTOR_LEFT_REAR);
  Motor_ResetPID(MOTOR_RIGHT_FRONT);
  Motor_ResetPID(MOTOR_LEFT_FRONT);

  Motor_SetOutputLimit(MOTOR_RIGHT_REAR, -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
  Motor_SetOutputLimit(MOTOR_LEFT_REAR,  -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
  Motor_SetOutputLimit(MOTOR_RIGHT_FRONT, -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
  Motor_SetOutputLimit(MOTOR_LEFT_FRONT,  -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));

  uint8_t source = motion_current.source;
  motion_active = 0U;

  /* 检查是否还有更多 motion 请求在队列中 */
  if (motion_queue_count > 0U)
  {
    /* 还有待执行的请求，不发送 STOPPED */
    return 1U;
  }

  if (source == 3)
  {
    UART3_SendText("STOPPED\r\n");
  }
  else
  {
    UART2_SendText("STOPPED\r\n");
  }

  return 1U;
}

/* steps >=1, dir = -1 左, +1 右, source 2=UART2 3=UART3 */
static void Mecanum_StepStrafe(int steps, int dir, uint8_t source)
{
  MotionRequest_t request;

  if (steps <= 0)
  {
    return;
  }

  request.kind = MOTION_KIND_STRAFE;
  request.steps = steps;
  request.dir = dir;
  request.source = source;
  (void)MotionQueue_Enqueue(&request);
}

/* steps >=1, dir = +1 forward, -1 backward, source 2=UART2 3=UART3 */
static void Mecanum_StepForward(int steps, int dir, uint8_t source)
{
  MotionRequest_t request;

  if (steps <= 0)
  {
    return;
  }

  request.kind = MOTION_KIND_FORWARD;
  request.steps = steps;
  request.dir = dir;
  request.source = source;
  (void)MotionQueue_Enqueue(&request);
}

/* steps >= 1, dir = +1 顺时针, -1 逆时针, source 2=UART2 3=UART3 */
static void Mecanum_StepCircle(int steps, int dir, uint8_t source)
{
  MotionRequest_t request;

  if (steps <= 0)
  {
    return;
  }

  request.kind = MOTION_KIND_CIRCLE;
  request.steps = steps;
  request.dir = dir;
  request.source = source;
  (void)MotionQueue_Enqueue(&request);
}

/* 判断是否有活跃的 motion（正在执行 + 队列等待中） */
static uint8_t IsMotionBusy(void)
{
  return (motion_active || (motion_queue_count > 0U)) ? 1U : 0U;
}

// 航向 PID 已移除；保留 Mecanum_SetMotion 声明
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

  /* 启动 UART3 接收 */
  if (HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart3_rx_frame, UART3_RX_FRAME_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

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
  /* Infinite loop - fixed 10ms periodic using vTaskDelayUntil */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(10);
  float dt_s = 0.01f;  /* fixed 10ms */

  /* Apply startup position PID output limit to prevent aggressive correction
   * at power-on. Each motor's position output limit is clamped to its own
   * POSITION_OUTPUT_LIMIT_STARTUP_xx (counts/sec). Adjust in config.h. */
  Motor_SetPositionOutputLimit(MOTOR_RIGHT_REAR,  POSITION_OUTPUT_LIMIT_STARTUP_RR);
  Motor_SetPositionOutputLimit(MOTOR_LEFT_REAR,   POSITION_OUTPUT_LIMIT_STARTUP_RL);
  Motor_SetPositionOutputLimit(MOTOR_RIGHT_FRONT, POSITION_OUTPUT_LIMIT_STARTUP_FR);
  Motor_SetPositionOutputLimit(MOTOR_LEFT_FRONT,  POSITION_OUTPUT_LIMIT_STARTUP_FL);

  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    (void)Motion_Tick();

    float heading_correction = 0.0f;
    uint8_t motion_busy_local = (base_speed_percent != 0.0f) || mecanum_mode_active || motion_active || Motor_HasActivePositionTarget();

    // 航向控制循环已移除 MPU6050 逻辑，保持 heading_correction = 0
    (void)heading_pitch_lpf_initialized;
    (void)heading_pitch_lpf;

    if (motion_busy_local)
    {
      Motor_SetTargetBiasPercent(MOTOR_RIGHT_REAR,  heading_correction);
      Motor_SetTargetBiasPercent(MOTOR_LEFT_REAR,  -heading_correction);
      Motor_SetTargetBiasPercent(MOTOR_RIGHT_FRONT, heading_correction);
      Motor_SetTargetBiasPercent(MOTOR_LEFT_FRONT,  -heading_correction);
    }
    else
    {
      Motor_ClearTargetBias(MOTOR_RIGHT_REAR);
      Motor_ClearTargetBias(MOTOR_LEFT_REAR);
      Motor_ClearTargetBias(MOTOR_RIGHT_FRONT);
      Motor_ClearTargetBias(MOTOR_LEFT_FRONT);
    }

    // 应用基础速度 + 航向校正差速
    if (!mecanum_mode_active && (base_speed_percent != 0.0f))
    {
      static float last_applied_percent = 0.0f;
      if (base_speed_percent != last_applied_percent)
      {
        Motor_SetTargetPercent(MOTOR_RIGHT_REAR, base_speed_percent);
        Motor_SetTargetPercent(MOTOR_LEFT_REAR, base_speed_percent);
        Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, base_speed_percent);
        Motor_SetTargetPercent(MOTOR_LEFT_FRONT, base_speed_percent);
        last_applied_percent = base_speed_percent;
      }
    }
    else if (mecanum_mode_active)
    {
      static float last_mecanum_targets[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
      static uint8_t mecanum_targets_dirty = 1;
      if ((mecanum_last_targets[MOTOR_RIGHT_REAR]  != last_mecanum_targets[MOTOR_RIGHT_REAR]) ||
          (mecanum_last_targets[MOTOR_LEFT_REAR]   != last_mecanum_targets[MOTOR_LEFT_REAR]) ||
          (mecanum_last_targets[MOTOR_RIGHT_FRONT] != last_mecanum_targets[MOTOR_RIGHT_FRONT]) ||
          (mecanum_last_targets[MOTOR_LEFT_FRONT]  != last_mecanum_targets[MOTOR_LEFT_FRONT]) ||
          mecanum_targets_dirty)
      {
        Motor_SetTargetPercent(MOTOR_RIGHT_REAR,  mecanum_last_targets[MOTOR_RIGHT_REAR]);
        Motor_SetTargetPercent(MOTOR_LEFT_REAR,   mecanum_last_targets[MOTOR_LEFT_REAR]);
        Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, mecanum_last_targets[MOTOR_RIGHT_FRONT]);
        Motor_SetTargetPercent(MOTOR_LEFT_FRONT,  mecanum_last_targets[MOTOR_LEFT_FRONT]);
        last_mecanum_targets[MOTOR_RIGHT_REAR]  = mecanum_last_targets[MOTOR_RIGHT_REAR];
        last_mecanum_targets[MOTOR_LEFT_REAR]   = mecanum_last_targets[MOTOR_LEFT_REAR];
        last_mecanum_targets[MOTOR_RIGHT_FRONT] = mecanum_last_targets[MOTOR_RIGHT_FRONT];
        last_mecanum_targets[MOTOR_LEFT_FRONT]  = mecanum_last_targets[MOTOR_LEFT_FRONT];
        mecanum_targets_dirty = 0;
      }
    }
    else
    {
      static float last_zero_percent = -1.0f;
      if (last_zero_percent != 0.0f)
      {
        Motor_SetTargetPercent(MOTOR_RIGHT_REAR, 0.0f);
        Motor_SetTargetPercent(MOTOR_LEFT_REAR, 0.0f);
        Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, 0.0f);
        Motor_SetTargetPercent(MOTOR_LEFT_FRONT, 0.0f);
        last_zero_percent = 0.0f;
      }
    }

    // 电机 PID 控制循环
    Motor_UpdateControl(dt_s);

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
  float rr = forward - strafe - rotation;
  float lr = forward + strafe + rotation;
  float rf = forward + strafe - rotation;
  float lf = forward - strafe + rotation;

  float max_speed = 0.0f;
  if (fabsf(rr) > max_speed) max_speed = fabsf(rr);
  if (fabsf(lr) > max_speed) max_speed = fabsf(lr);
  if (fabsf(rf) > max_speed) max_speed = fabsf(rf);
  if (fabsf(lf) > max_speed) max_speed = fabsf(lf);

  if (max_speed > 100.0f)
  {
    float scale = 100.0f / max_speed;
    rr *= scale;
    lr *= scale;
    rf *= scale;
    lf *= scale;
  }

  base_speed_percent = (fabsf(forward) + fabsf(strafe) + fabsf(rotation)) / 3.0f;

  Motor_ResetPID(MOTOR_RIGHT_REAR);
  Motor_ResetPID(MOTOR_LEFT_REAR);
  Motor_ResetPID(MOTOR_RIGHT_FRONT);
  Motor_ResetPID(MOTOR_LEFT_FRONT);

  float max_percent = COMMAND_MAX_OUTPUT_PERCENT;
  if (rr > max_percent) rr = max_percent; else if (rr < -max_percent) rr = -max_percent;
  if (lr > max_percent) lr = max_percent; else if (lr < -max_percent) lr = -max_percent;
  if (rf > max_percent) rf = max_percent; else if (rf < -max_percent) rf = -max_percent;
  if (lf > max_percent) lf = max_percent; else if (lf < -max_percent) lf = -max_percent;

  float pwm_limit_local = (COMMAND_MAX_OUTPUT_PERCENT / 100.0f) * MOTOR_PWM_PERIOD;
  Motor_SetOutputLimit(MOTOR_RIGHT_REAR,  -pwm_limit_local, pwm_limit_local);
  Motor_SetOutputLimit(MOTOR_LEFT_REAR,   -pwm_limit_local, pwm_limit_local);
  Motor_SetOutputLimit(MOTOR_RIGHT_FRONT, -pwm_limit_local, pwm_limit_local);
  Motor_SetOutputLimit(MOTOR_LEFT_FRONT,  -pwm_limit_local, pwm_limit_local);

  Motor_SetTargetPercent(MOTOR_RIGHT_REAR, rr);
  Motor_SetTargetPercent(MOTOR_LEFT_REAR, lr);
  Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, rf);
  Motor_SetTargetPercent(MOTOR_LEFT_FRONT, lf);

  mecanum_last_targets[MOTOR_RIGHT_REAR]  = rr;
  mecanum_last_targets[MOTOR_LEFT_REAR]   = lr;
  mecanum_last_targets[MOTOR_RIGHT_FRONT] = rf;
  mecanum_last_targets[MOTOR_LEFT_FRONT]  = lf;
  mecanum_mode_active = 1;
}

/* Heading PID and associated low-pass filter removed; functionality deprecated. */

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

static void UART3_SendText(const char *text)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
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

  if (byte == '{')
  {
    uart2_in_array = 1;
    uart2_rx_len = 0;
    return;
  }
  if (byte == '}')
  {
    if (uart2_rx_len > 0U)
    {
      uart2_rx_line[uart2_rx_len] = '\0';
      strncpy(uart2_cmd_queue[uart2_cmd_tail], uart2_rx_line, UART2_CMD_MAX_LEN - 1U);
      uart2_cmd_queue[uart2_cmd_tail][UART2_CMD_MAX_LEN - 1U] = '\0';
      uart2_cmd_tail = (uint8_t)((uart2_cmd_tail + 1U) % UART2_CMD_QUEUE_DEPTH);
      uart2_cmd_count++;
      uart2_rx_len = 0;
    }
    uart2_in_array = 0;
    return;
  }
  if (byte == ',')
  {
    if (uart2_in_array && (uart2_rx_len > 0U))
    {
      uart2_rx_line[uart2_rx_len] = '\0';
      strncpy(uart2_cmd_queue[uart2_cmd_tail], uart2_rx_line, UART2_CMD_MAX_LEN - 1U);
      uart2_cmd_queue[uart2_cmd_tail][UART2_CMD_MAX_LEN - 1U] = '\0';
      uart2_cmd_tail = (uint8_t)((uart2_cmd_tail + 1U) % UART2_CMD_QUEUE_DEPTH);
      uart2_cmd_count++;
      uart2_rx_len = 0;
    }
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
  if (uart2_in_array)
  {
    return;
  }

  if (uart2_rx_len > 0U)
  {
    uart2_rx_line[uart2_rx_len] = '\0';
    UART2_EnqueueCommandFromISR(uart2_rx_line);
    uart2_rx_len = 0U;
  }
}

/* UART3 指令处理 */

static void UART3_HandleCommand(const char *command)
{
  const char *cursor = command;

  while ((*cursor == ' ') || (*cursor == '\t'))
    cursor++;

  /* STOP 命令不受互斥限制 */
  if (strncmp(cursor, "STOP", 4) == 0)
  {
    cursor += 4;
    while ((*cursor == ' ') || (*cursor == '\t'))
      cursor++;
    if (*cursor != '\0')
      return;

    base_speed_percent = 0.0f;
    mecanum_mode_active = 0;
    MotionQueue_Clear();
    motion_active = 0U;
    Motor_ClearPositionTarget(MOTOR_RIGHT_REAR);
    Motor_ClearPositionTarget(MOTOR_LEFT_REAR);
    Motor_ClearPositionTarget(MOTOR_RIGHT_FRONT);
    Motor_ClearPositionTarget(MOTOR_LEFT_FRONT);
    Motor_SetTargetPercent(MOTOR_RIGHT_REAR, 0.0f);
    Motor_SetTargetPercent(MOTOR_LEFT_REAR, 0.0f);
    Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, 0.0f);
    Motor_SetTargetPercent(MOTOR_LEFT_FRONT, 0.0f);
    Motor_StopAll();
    Motor_ResetPID(MOTOR_RIGHT_REAR);
    Motor_ResetPID(MOTOR_LEFT_REAR);
    Motor_ResetPID(MOTOR_RIGHT_FRONT);
    Motor_ResetPID(MOTOR_LEFT_FRONT);
    Motor_SetOutputLimit(MOTOR_RIGHT_REAR, -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
    Motor_SetOutputLimit(MOTOR_LEFT_REAR,  -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
    Motor_SetOutputLimit(MOTOR_RIGHT_FRONT, -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
    Motor_SetOutputLimit(MOTOR_LEFT_FRONT,  -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
    return;
  }

  /* CIRCLE n 指令 — 受互斥保护 */
  if (strncmp(cursor, "CIRCLE", 6) == 0)
  {
    if (IsMotionBusy())
      return;

    cursor += 6;
    while ((*cursor == ' ') || (*cursor == '\t'))
      cursor++;

    int steps = 1;
    if (*cursor != '\0')
    {
      char *endp = NULL;
      long parsed = strtol(cursor, &endp, 10);
      if ((endp == cursor) || (parsed <= 0))
        return;
      while ((*endp == ' ') || (*endp == '\t'))
        endp++;
      if (*endp != '\0')
        return;
      steps = (int)parsed;
    }

    Mecanum_StepCircle(steps, +1, 3);
    return;
  }
}

static void UART3_ProcessByte(uint8_t byte)
{
  if ((byte == '\r') || (byte == '\n'))
  {
    if (uart3_rx_len > 0U)
    {
      uart3_rx_line[uart3_rx_len] = '\0';
      UART3_HandleCommand(uart3_rx_line);
      uart3_rx_len = 0;
    }
    return;
  }

  if (uart3_rx_len < (UART3_CMD_MAX_LEN - 1U))
  {
    uart3_rx_line[uart3_rx_len++] = (char)byte;
  }
  else
  {
    uart3_rx_len = 0U;
  }
}

static void UART2_HandleCommand(const char *command)
{
  const char *cursor = command;

  while ((*cursor == ' ') || (*cursor == '\t'))
  {
    cursor++;
  }

  if (strncmp(cursor, "STOP", 4) == 0)
  {
    cursor += 4;
    while ((*cursor == ' ') || (*cursor == '\t'))
    {
      cursor++;
    }

    if (*cursor != '\0')
    {
      return;
    }

    base_speed_percent = 0.0f;
    mecanum_mode_active = 0;
    MotionQueue_Clear();
    motion_active = 0U;
    mecanum_last_targets[MOTOR_RIGHT_REAR] = 0.0f;
    mecanum_last_targets[MOTOR_LEFT_REAR] = 0.0f;
    mecanum_last_targets[MOTOR_RIGHT_FRONT] = 0.0f;
    mecanum_last_targets[MOTOR_LEFT_FRONT] = 0.0f;
    Motor_ClearPositionTarget(MOTOR_RIGHT_REAR);
    Motor_ClearPositionTarget(MOTOR_LEFT_REAR);
    Motor_ClearPositionTarget(MOTOR_RIGHT_FRONT);
    Motor_ClearPositionTarget(MOTOR_LEFT_FRONT);
    Motor_SetTargetPercent(MOTOR_RIGHT_REAR, 0.0f);
    Motor_SetTargetPercent(MOTOR_LEFT_REAR, 0.0f);
    Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, 0.0f);
    Motor_SetTargetPercent(MOTOR_LEFT_FRONT, 0.0f);
    Motor_StopAll();
    Motor_ResetPID(MOTOR_RIGHT_REAR);
    Motor_ResetPID(MOTOR_LEFT_REAR);
    Motor_ResetPID(MOTOR_RIGHT_FRONT);
    Motor_ResetPID(MOTOR_LEFT_FRONT);
    Motor_SetOutputLimit(MOTOR_RIGHT_REAR, -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
    Motor_SetOutputLimit(MOTOR_LEFT_REAR,  -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
    Motor_SetOutputLimit(MOTOR_RIGHT_FRONT, -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
    Motor_SetOutputLimit(MOTOR_LEFT_FRONT,  -(MOTOR_PWM_PERIOD), (MOTOR_PWM_PERIOD));
    UART2_SendText("STOPPED\r\n");
    return;
  }

  if (strncmp(cursor, "LEFT", 4) == 0)
  {
    int steps = 1;
    cursor += 4;
    while ((*cursor == ' ') || (*cursor == '\t'))
    {
      cursor++;
    }
    if (*cursor != '\0')
    {
      char *end_ptr_local = NULL;
      long parsed = strtol(cursor, &end_ptr_local, 10);
      if ((end_ptr_local == cursor) || (parsed <= 0))
      {
        return;
      }
      while ((*end_ptr_local == ' ') || (*end_ptr_local == '\t'))
      {
        end_ptr_local++;
      }
      if (*end_ptr_local != '\0')
      {
        return;
      }
      steps = (int)parsed;
    }
    Mecanum_StepStrafe(steps, +1, 2);
    return;
  }

  if (strncmp(cursor, "RIGHT", 5) == 0)
  {
    int steps = 1;
    cursor += 5;
    while ((*cursor == ' ') || (*cursor == '\t'))
    {
      cursor++;
    }
    if (*cursor != '\0')
    {
      char *end_ptr_local = NULL;
      long parsed = strtol(cursor, &end_ptr_local, 10);
      if ((end_ptr_local == cursor) || (parsed <= 0))
      {
        return;
      }
      while ((*end_ptr_local == ' ') || (*end_ptr_local == '\t'))
      {
        end_ptr_local++;
      }
      if (*end_ptr_local != '\0')
      {
        return;
      }
      steps = (int)parsed;
    }
    Mecanum_StepStrafe(steps, -1, 2);
    return;
  }

  if ((strncmp(cursor, "FORWARD", 7) == 0) || (strncmp(cursor, "BACKWARD", 8) == 0) || (strncmp(cursor, "BACK", 4) == 0))
  {
    int steps = 1;
    int direction = +1;

    if (strncmp(cursor, "BACKWARD", 8) == 0)
    {
      cursor += 8;
      direction = -1;
    }
    else if (strncmp(cursor, "BACK", 4) == 0)
    {
      cursor += 4;
      direction = -1;
    }
    else
    {
      cursor += 7;
    }

    while ((*cursor == ' ') || (*cursor == '\t'))
    {
      cursor++;
    }
    if (*cursor != '\0')
    {
      char *end_ptr_local = NULL;
      long parsed = strtol(cursor, &end_ptr_local, 10);
      if ((end_ptr_local == cursor) || (parsed <= 0))
      {
        return;
      }
      while ((*end_ptr_local == ' ') || (*end_ptr_local == '\t'))
      {
        end_ptr_local++;
      }
      if (*end_ptr_local != '\0')
      {
        return;
      }
      steps = (int)parsed;
    }
    Mecanum_StepForward(steps, direction, 2);
    return;
  }

  if (strncmp(cursor, "CIRCLE", 6) == 0)
  {
    int steps = 1;
    cursor += 6;
    while ((*cursor == ' ') || (*cursor == '\t'))
    {
      cursor++;
    }
    if (*cursor != '\0')
    {
      char *end_ptr_local = NULL;
      long parsed = strtol(cursor, &end_ptr_local, 10);
      if ((end_ptr_local == cursor) || (parsed <= 0))
      {
        return;
      }
      while ((*end_ptr_local == ' ') || (*end_ptr_local == '\t'))
      {
        end_ptr_local++;
      }
      if (*end_ptr_local != '\0')
      {
        return;
      }
      steps = (int)parsed;
    }
    Mecanum_StepCircle(steps, +1, 2);
    return;
  }

  if (strncmp(cursor, "RUN", 3) == 0)
  {
    cursor += 3;
    while ((*cursor == ' ') || (*cursor == '\t'))
    {
      cursor++;
    }

    if (*cursor == '\0')
    {
      return;
    }

    char *end_ptr = NULL;
    long parsed = strtol(cursor, &end_ptr, 10);
    if ((end_ptr == cursor) || (parsed < -100L) || (parsed > 100L))
    {
      return;
    }
    while ((*end_ptr == ' ') || (*end_ptr == '\t'))
    {
      end_ptr++;
    }
    if (*end_ptr != '\0')
    {
      return;
    }

    base_speed_percent = (float)parsed;
    mecanum_mode_active = 0;
    Motor_ResetPID(MOTOR_RIGHT_REAR);
    Motor_ResetPID(MOTOR_LEFT_REAR);
    Motor_ResetPID(MOTOR_RIGHT_FRONT);
    Motor_ResetPID(MOTOR_LEFT_FRONT);
    if (base_speed_percent > COMMAND_MAX_OUTPUT_PERCENT) base_speed_percent = COMMAND_MAX_OUTPUT_PERCENT;
    if (base_speed_percent < -COMMAND_MAX_OUTPUT_PERCENT) base_speed_percent = -COMMAND_MAX_OUTPUT_PERCENT;

    float pwm_limit = (COMMAND_MAX_OUTPUT_PERCENT / 100.0f) * MOTOR_PWM_PERIOD;
    Motor_SetOutputLimit(MOTOR_RIGHT_REAR,  -pwm_limit, pwm_limit);
    Motor_SetOutputLimit(MOTOR_LEFT_REAR,   -pwm_limit, pwm_limit);
    Motor_SetOutputLimit(MOTOR_RIGHT_FRONT, -pwm_limit, pwm_limit);
    Motor_SetOutputLimit(MOTOR_LEFT_FRONT,  -pwm_limit, pwm_limit);

    Motor_SetTargetPercent(MOTOR_RIGHT_REAR, base_speed_percent);
    Motor_SetTargetPercent(MOTOR_LEFT_REAR, base_speed_percent);
    Motor_SetTargetPercent(MOTOR_RIGHT_FRONT, base_speed_percent);
    Motor_SetTargetPercent(MOTOR_LEFT_FRONT, base_speed_percent);
    return;
  }

  if (strncmp(cursor, "MPU", 3) == 0)
  {
    UART2_SendText("MPU functionality disabled in this build\r\n");
    return;
  }

  if (strncmp(cursor, "PID", 3) == 0)
  {
    cursor += 3;
    while ((*cursor == ' ') || (*cursor == '\t'))
    {
      cursor++;
    }

    char *endp = NULL;
    float kp = strtof(cursor, &endp);
    if (endp == cursor)
    {
      return;
    }

    cursor = endp;
    while ((*cursor == ' ') || (*cursor == '\t'))
    {
      cursor++;
    }

    float ki = strtof(cursor, &endp);
    if (endp == cursor)
    {
      return;
    }

    cursor = endp;
    while ((*cursor == ' ') || (*cursor == '\t'))
    {
      cursor++;
    }

    float kd = strtof(cursor, &endp);
    if (endp == cursor)
    {
      return;
    }

    while ((*endp == ' ') || (*endp == '\t'))
    {
      endp++;
    }
    if (*endp != '\0')
    {
      return;
    }

    (void)kp; (void)ki; (void)kd;
    UART2_SendText("Heading PID removed in this build\r\n");
    return;
  }

  return;
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

  if (huart->Instance == USART3)
  {
    for (uint16_t index = 0U; index < Size; index++)
    {
      UART3_ProcessByte(uart3_rx_frame[index]);
    }

    if (HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart3_rx_frame, UART3_RX_FRAME_SIZE) != HAL_OK)
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

  if (huart->Instance == USART3)
  {
    uart3_rx_len = 0U;
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart3_rx_frame, UART3_RX_FRAME_SIZE);
  }
}

/* USER CODE END 4 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */