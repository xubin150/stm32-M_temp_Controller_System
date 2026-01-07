/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osMessageQueueId_t UartQueueHandle; // 串口打印队列

// ====================================================
// 新增：应用状态机定义
// ====================================================
typedef enum {
    STATE_OFF = 0,             // 初始状态，未加热，可调整目标温度 SV
   // STATE_SV_SETTING = 0,      // 与 STATE_OFF 相同，默认是设置模式
    STATE_HEAT_READY,          // PV 显示 "HEAT"，等待 OK 启动
    STATE_AUTOTUNE_READY,      // PV 显示 "ATUNE"，等待 OK 启动
    STATE_HEATING,             // PID 正在运行，控制加热
    STATE_AUTOTUNE_RUNNING,    // 自整定正在运行 (Save_Task 负责)
    STATE_FAULT                // MAX31855 传感器故障
} AppState_t;

/* Helper struct for EEPROM storage */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Setpoint; // 新增：目标温度设置值
    uint8_t magic;
} PID_Save_t;

typedef enum {
    MODE_POWER = 0,    // 功率显示格式
    MODE_VOLTAGE       // 电压显示格式
} DisplayMode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AVG_N 8


#define EEPROM_PID_ADDR 0x00
#define PID_MAGIC 0xA6

#define MAX_SAFE_TEMP_C 200.0f // 整定时与运行时的安全温度上限（请按实际设备设置）

// TM1652 通道定义 (对应 bsp_tm1652.c)
#define TM1652_CHANNEL_PV 0 // Process Value - 当前温度
#define TM1652_CHANNEL_SV 1 // Set Value - 目标温度
#define TM1652_CHANNEL_POW 3 // Power - 功率 (W) <-- 新增：Channel 2
#define TM1652_CHANNEL_VOL 2 // Voltage - 电压 (V) <-- 新增：Channel 3
// 数码管段码常量
#define SEG_DP_MASK 0x80 // DP点在最高位
#define TM1652_SET_CONTROL_LOW   0x18 //段驱动电流
#define TM1652_SET_CONTROL_MID   0x1E //段驱动电流1/2
#define TM1652_SET_CONTROL_MAX   0xFE //段驱动电流
#define KEY_MENU_LONG_PRESS_MS 5000 // 5秒长按进入自整定
#define KEY_ACCEL_PRESS_MS 200 // 500ms 长按进入加速调节
#define ACCEL_STEP_SIZE 10.0f // 加速调节步长 10.0度

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

float tempBuffer[AVG_N];
uint8_t tempIndex = 0;
uint8_t tempCount = 0;

float   PID_Input_Temperature = 0.0f;

/* PID 对象（在 pid.c 中实现） */
PID_HandleTypeDef HeaterPID;
uint8_t g_pid_valid = 0; // EEPROM 有效标识

/* autotune / save flags  自整定标志位 */
volatile uint8_t g_autotune_request = 0;
volatile uint8_t g_save_pid_request = 0;

static uint8_t pv_reached_sv = 0;
/* synchronization */
osMutexId_t  xMutexTemp;
osMutexId_t  xMutexPID;
osMutexId_t xMutexPower; // 用于保护 HLW8032_GetData() 返回值的读取一致性

osMutexId_t xMutexState; // 新增：状态机和SV互斥锁

// ** 蜂鸣器信号量 **
osSemaphoreId_t beep_signal_sem; // 声明信号量句柄，供 bsp_io.c 引用

/* shared setpoint (可在 UI/按键中修改) */
float g_setpoint = 100.0f;

// 全局应用状态
volatile AppState_t g_app_state = STATE_OFF;

// 用于指示当前是否在调节 SV 设定值（且在 STATE_OFF 状态下）
volatile uint8_t g_is_sv_adjusting = 0;

// 0: 未静音，1: 已静音
volatile uint8_t g_fault_muted = 0;

extern uint8_t RxFrameCopy[HLW8032_FRAME_SIZE];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for Temp */
osThreadId_t TempHandle;
const osThreadAttr_t Temp_attributes = {
  .name = "Temp",
  .priority = (osPriority_t) osPriorityNormal3,
  .stack_size = 256 * 4
};
/* Definitions for Heat */
osThreadId_t HeatHandle;
const osThreadAttr_t Heat_attributes = {
  .name = "Heat",
  .priority = (osPriority_t) osPriorityNormal4,
  .stack_size = 256 * 4
};
/* Definitions for Power */
osThreadId_t PowerHandle;
const osThreadAttr_t Power_attributes = {
  .name = "Power",
  .priority = (osPriority_t) osPriorityNormal3,
  .stack_size = 512 * 4
};
/* Definitions for UI */
osThreadId_t UIHandle;
const osThreadAttr_t UI_attributes = {
  .name = "UI",
  .priority = (osPriority_t) osPriorityNormal3,
  .stack_size = 256 * 4
};
/* Definitions for Key */
osThreadId_t KeyHandle;
const osThreadAttr_t Key_attributes = {
  .name = "Key",
  .priority = (osPriority_t) osPriorityNormal3,
  .stack_size = 256 * 4
};
/* Definitions for Save */
osThreadId_t SaveHandle;
const osThreadAttr_t Save_attributes = {
  .name = "Save",
  .priority = (osPriority_t) osPriorityBelowNormal1,
  .stack_size = 1024 * 4
};
/* Definitions for Beep */
osThreadId_t BeepHandle;
const osThreadAttr_t Beep_attributes = {
  .name = "Beep",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for UART_Print */
osThreadId_t UART_PrintHandle;
const osThreadAttr_t UART_Print_attributes = {
  .name = "UART_Print",
  .priority = (osPriority_t) osPriorityLow7,
  .stack_size = 128 * 4
};
/* Definitions for Fault_detector */
osThreadId_t Fault_detectorHandle;
const osThreadAttr_t Fault_detector_attributes = {
  .name = "Fault_detector",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static float Temp_Filter(float newVal);
static int PID_LoadFromEEPROM(void);
//static int PID_SaveToEEPROM(void);
 float read_temp_impl(void);
 void set_heater_impl(uint8_t on); // power: 0..1

 // TM1652 驱动接口声明 (需要确保 bsp_tm1652.c 被编译链接)
 // SegmentMap 应该包含 0-9, 减号 (10), 消隐 (11)
 extern const uint8_t TM1652_SegmentMap[];
 // 温度转换函数
 static void FloatToSegments(float temp, uint8_t output[4]);
 void Convert_Float_To_Segments_Formatted(float value, uint8_t out_seg[4], DisplayMode_t mode);


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Temp_Task(void *argument);
void Heat_Task(void *argument);
void Power_Task(void *argument);
void UI_Task(void *argument);
void Key_Task(void *argument);
void Save_Task(void *argument);
void Beep_Task(void *argument);
void UART_Print_Task(void *argument);
void Fault_detector_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	 /* create mutexes */
	  xMutexTemp = osMutexNew(NULL);
	  xMutexPID  = osMutexNew(NULL);
	  xMutexPower = osMutexNew(NULL); // 初始化功率互斥锁
	  xMutexState = osMutexNew(NULL); // 初始化状态机互斥锁
	  // ** 初始化蜂鸣器信号量 **
	  // 使用计数信号量，最大计数5，初始计数0
	  beep_signal_sem = osSemaphoreNew(5, 0, NULL);
	  /* load PID from EEPROM before creating tasks so Heat_Task can know g_pid_valid immediately */

	  if (PID_LoadFromEEPROM() == 0) {
			// Initialize PID controller with loaded params
			PID_Init(&HeaterPID, HeaterPID.Kp, HeaterPID.Ki, HeaterPID.Kd, 0.0f, 1.0f);
			g_pid_valid = 1;
		} else {
			// mark invalid and init PID with safe zeros
			g_pid_valid = 0;
			PID_Init(&HeaterPID, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
			// 假设默认目标温度为 100
			g_setpoint = 100.0f;
		}
	    HeaterPID.Kp = 0.12;   // 参数P  //300W功率加热棒 目标温度150 ，145度时功率明显下降
	    HeaterPID.Ki = 0.0006;  // 参数I 0.002超温3度，且来回震荡  ->调小 振幅减小
	    HeaterPID.Kd = 0.025; //
	    // 0.12  0.0008  0.02 处于轻微振荡状态 处于切阻尼
	    // 0.12 0.0006 0.025 温度会回落1度左右，然后缓慢升到目标温度 目标温度较高时需要很长时间到达目标温度


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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
 /* 创建串口打印队列，16 条消息，每条 128 字节 */
	UartQueueHandle = osMessageQueueNew(16, 128, NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Temp */
  TempHandle = osThreadNew(Temp_Task, NULL, &Temp_attributes);

  /* creation of Heat */
  HeatHandle = osThreadNew(Heat_Task, NULL, &Heat_attributes);

  /* creation of Power */
  PowerHandle = osThreadNew(Power_Task, NULL, &Power_attributes);

  /* creation of UI */
  UIHandle = osThreadNew(UI_Task, NULL, &UI_attributes);

  /* creation of Key */
  KeyHandle = osThreadNew(Key_Task, NULL, &Key_attributes);

  /* creation of Save */
  SaveHandle = osThreadNew(Save_Task, NULL, &Save_attributes);

  /* creation of Beep */
  BeepHandle = osThreadNew(Beep_Task, NULL, &Beep_attributes);

  /* creation of UART_Print */
  UART_PrintHandle = osThreadNew(UART_Print_Task, NULL, &UART_Print_attributes);

  /* creation of Fault_detector */
  Fault_detectorHandle = osThreadNew(Fault_detector_Task, NULL, &Fault_detector_attributes);

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
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Temp_Task */
/**
* @brief Function implementing the Temp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Temp_Task */
void Temp_Task(void *argument)
{
  /* USER CODE BEGIN Temp_Task */

  uint32_t nextWakeTime = osKernelGetTickCount();
  const uint32_t period = TEMP_PERIOD_MS; // 250ms
  char msg[128];
 // AppState_t old_state = STATE_OFF; // 用于检测状态变化
  /* Infinite loop */
  for(;;)
  {
   //1. 读取温度数据
   MAX31855_ReadData(&MAX31855_Handle);
   uint8_t current_fault = MAX31855_GetFault(&MAX31855_Handle);
   uint8_t over_temp_fault = 0;
   float raw_temp = 0.0f;
   float filtered_temp = 0.0f;

   if(!current_fault)
	 {
	     raw_temp = MAX31855_GetTemperature(&MAX31855_Handle);
	     filtered_temp = Temp_Filter(raw_temp); // 移动平均滤波
		 //2. 放入 PID 共享变量（线程安全）
		 if (osMutexAcquire(xMutexTemp, osWaitForever) == osOK) {
		     PID_Input_Temperature = filtered_temp;
		     osMutexRelease(xMutexTemp);
		 }

	 // 3. 检查安全温度
	 if (filtered_temp > g_setpoint+10) { //超温10摄氏度  报警
		 over_temp_fault = 1;
	 }
		 snprintf(msg, sizeof(msg), "Temperature: %.1f C\r\n", filtered_temp);
		 // 发送到 UART 打印队列
	     osMessageQueuePut(UartQueueHandle, &msg, 0, pdMS_TO_TICKS(10));
	 }
   // 4. 故障状态机管理
   	 if ( over_temp_fault) //删除current_fault
   	 {
   	     // 发生故障，强制进入故障模式并关闭加热
   	     if (osMutexAcquire(xMutexState, osWaitForever) == osOK) {
   	      //   old_state = g_app_state;
   	         g_app_state = STATE_FAULT;
   	         // MARK: Optimization 1: 进入故障模式时重置静音状态
   	         g_fault_muted = 0;
   	         osMutexRelease(xMutexState);
   	     }
   	     set_heater_impl(0);
   	     LED_Red_Set(1); // 故障红灯常亮

   	     // 只有在未静音时才触发蜂鸣器
   	     if (!g_fault_muted) {
   	         Beep_Start(); // 报警
   	     }

   	     if (current_fault) {
   	    	 snprintf(msg, sizeof(msg), "MAX31855 Fault! Raw=%.1f\r\n", raw_temp);
   	     } else {
   	    	 snprintf(msg, sizeof(msg), "OVER TEMP Fault! T=%.1f\r\n", filtered_temp);
   	     }
   	     // MARK: Optimization 8: UART 打印队列使用短时等待
   	     osMessageQueuePut(UartQueueHandle, &msg, 0, pdMS_TO_TICKS(10));

   	 }else {
	     // 故障解除后，如果当前状态是 FAULT，则切换回 OFF 状态
	     if (osMutexAcquire(xMutexState, osWaitForever) == osOK) {
	         if (g_app_state == STATE_FAULT) {
	             g_app_state = STATE_OFF;
	             // 故障解除，重置静音和指示灯状态
	             g_fault_muted = 0;
	             LED_Red_Set(0);
	             Beep_Start(); // 提示音
	         }
	         osMutexRelease(xMutexState);
	     }
	     // 持续处于非故障状态时，可以熄灭红灯（如果是Key_Task静音后留下的红灯）
	     if (g_app_state != STATE_FAULT) {
	         LED_Red_Set(0);
	     }
	 }

   // 精确周期延时
          nextWakeTime += period;
          osDelayUntil(nextWakeTime);
   // osDelay(1);
  }
  /* USER CODE END Temp_Task */
}

/* USER CODE BEGIN Header_Heat_Task */
/**
* @brief Function implementing the Heat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Heat_Task */
void Heat_Task(void *argument)
{
  /* USER CODE BEGIN Heat_Task */
	uint32_t nextWakeTime = osKernelGetTickCount();
	const uint32_t period = PID_PERIOD_MS; //500ms
	float temp = 0.0;
	AppState_t current_state;
	static uint8_t tpc_counter = 0;
	static uint8_t target_on_steps = 0;
  /* Infinite loop */
  for(;;)
  {

      // 1. 获取当前状态和温度
	  if (osMutexAcquire(xMutexState, osWaitForever) == osOK) {
	      current_state = g_app_state;
	      osMutexRelease(xMutexState);
	  }
	  if (current_state == STATE_AUTOTUNE_RUNNING) {
	      // Heat_Task 完全放弃控制，由 autotune 专用逻辑接管
	      nextWakeTime += period;
	      osDelayUntil(nextWakeTime);
	      continue;
	  }

	  if (osMutexAcquire(xMutexTemp, osWaitForever) == osOK) {
		  temp = PID_Input_Temperature;
		  osMutexRelease(xMutexTemp);
	  }
	  // 2. 只有在 HEATING 状态下才执行 PID 逻辑
	   if (current_state == STATE_HEATING)
	   {

		  // 2.1 安全检查：PID 参数必须有效
		  if (!g_pid_valid) {
			  set_heater_impl(0);
			  tpc_counter = 0;
			  char msg[64];
			  snprintf(msg, sizeof(msg), "PID params INVALID! Heater disabled.\r\n");
			  osMessageQueuePut(UartQueueHandle, &msg, 0, pdMS_TO_TICKS(10));

			  nextWakeTime += period;
			  osDelayUntil(nextWakeTime);
			  continue;
		  }
		  // 2.2 PID 计算
		   float dt = (float)period / 1000.0f;
		   float power_ratio = 0.0f; // 0.0 - 1.0

		   // 使用互斥锁保护 PID 结构体和 setpoint
		   if (osMutexAcquire(xMutexPID, 5) == osOK) {
			   power_ratio = PID_Compute(&HeaterPID, g_setpoint, temp, dt);
			   osMutexRelease(xMutexPID);
		   }

		   if (power_ratio < 0.0f) power_ratio = 0.0f;
		   if (power_ratio > 1.0f) power_ratio = 1.0f;

		   // 2.3 TPC 步数计算 在 TPC 周期开始 (tpc_counter == 0) 时更新目标步数
		   if (tpc_counter == 0)
		   {
			   target_on_steps = (uint8_t)(power_ratio * TPC_STEPS + 0.5f);
			   if (target_on_steps > TPC_STEPS) target_on_steps = TPC_STEPS;
		   }
		   // 2.4 TPC 控制输出
		   if(tpc_counter < target_on_steps)
		   {
			   set_heater_impl(1);  // SSR ON
		//	   LED_Green_Set(1);
		   }

		   else
		   {
			   set_heater_impl(0);  // SSR OFF
			//   LED_Green_Set(0);
		   }


		   // 2.5 更新计数器
		   tpc_counter++;
		   if(tpc_counter >= TPC_STEPS)
			   tpc_counter = 0;

		   // 2.6 打印调试信息
			char msg[128];
			snprintf(msg, sizeof(msg), "HEAT: T=%.1f, Ratio=%.2f, TPC Step=%d, Target=%d\r\n",
					 temp,
					 power_ratio,
					 (int)tpc_counter,
					 (int)target_on_steps);
			osMessageQueuePut(UartQueueHandle, &msg, 0, pdMS_TO_TICKS(10));

	   }
	  else
	  {

		  // 3. 非 HEATING 状态，关闭加热

          // 排除 STATE_AUTOTUNE_RUNNING 状态，此时加热由 Save_Task 中的自整定逻辑控制。
	      if (current_state != STATE_AUTOTUNE_RUNNING) {
	          set_heater_impl(0);
	          tpc_counter = 0; // 重置 TPC
	      }
	  }

	 nextWakeTime += period;
	 osDelayUntil(nextWakeTime);
  }
  /* USER CODE END Heat_Task */
}

/* USER CODE BEGIN Header_Power_Task */
/**
* @brief Function implementing the Power thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Power_Task */
void Power_Task(void *argument)
{
  /* USER CODE BEGIN Power_Task */
//	uint8_t frame[24];
//	uint8_t idx = 0;
//	uint32_t startTime = 0;
	// uint32_t nextWakeTime = osKernelGetTickCount();
	// const uint32_t period = 250; // 250ms 周期
  /* Infinite loop */
  for(;;)
  {
	//  uint8_t byte;
      // 每次阻塞接收 1 字节，超时 5ms
      if (HAL_UART_Receive(&huart3, RxFrameCopy, 24, 100) == HAL_OK)
      {

        	  HLW8032_ProcessAndStore();

      //        HLW8032_Value_t data = HLW8032_GetData();
/*              char msg[128];
              snprintf(msg, sizeof(msg),
                       "POWER: V=%.1fV A=%.3fA P=%.1fW\r\n",
                       data.Voltage, data.Current, data.Power);
              osMessageQueuePut(UartQueueHandle, msg, 0, 10);*/



      }

      osDelay(10);
  }
  /* USER CODE END Power_Task */
}

/* USER CODE BEGIN Header_UI_Task */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_Task */
void UI_Task(void *argument)
{
  /* USER CODE BEGIN UI_Task */
	 LED_Green_Set(1);
	 LED_Red_Set(1);
	 osDelay(1000);
	 uint32_t nextWakeTime = osKernelGetTickCount();
	const uint32_t period =250; // 250ms 刷新周期
	float pv_temp = 0.0f;
	float sv_temp = 0.0f;
	float power = 0.0f;
	float voltage = 0.0f;

	static float pv_display = 0.0f;   // 用于显示的温度
	static float pv_hold = 0.0f;      // 锁定显示温度
	static uint8_t pv_display_init = 0;
	static uint8_t pv_hold_active = 0;



	uint8_t pv_data[4];
	uint8_t sv_data[4];
	uint8_t pow_data[4];
	uint8_t vol_data[4];
	HLW8032_Value_t power_data;
	AppState_t current_state;
	uint8_t is_sv_adjusting = 0; // 局部变量
	// 模式文字 "HEAT" (H E A t) 和 "ATUN" (A t u n) 的段码定义
	const uint8_t SEG_HEAT[] = {0x76, 0x79, 0x77, 0x78}; // H E A T -> (H, E, A, t)
	const uint8_t SEG_ATUN[] = {0x77, 0x78, 0x3e, 0x54}; // A T U N -> (A, t, u, n)
	const uint8_t SEG_BLANK[] = {0x00, 0x00, 0x00, 0x00}; // 全灭
  /* Infinite loop */
  for(;;)
  {
	  // 1. 读取共享数据和状态 (使用互斥锁)
	  if (osMutexAcquire(xMutexState, osWaitForever) == osOK) {
		  current_state = g_app_state;
		  sv_temp = g_setpoint;
		  // MARK: Optimization 2: 读取 SV 调节标志
		  is_sv_adjusting = g_is_sv_adjusting;
		  osMutexRelease(xMutexState);
	  }
	  if (osMutexAcquire(xMutexTemp, osWaitForever) == osOK) {
		  pv_temp = PID_Input_Temperature;
		  osMutexRelease(xMutexTemp);
	    }
	  if (osMutexAcquire(xMutexPower, osWaitForever) == osOK) {
		  power_data = HLW8032_GetData();
		  osMutexRelease(xMutexPower);
	  }
	  power = power_data.Power;
	  voltage = power_data.Voltage;

	  if (!pv_display_init)
	  {
	      pv_display = pv_temp;
	      pv_hold = pv_temp;
	      pv_display_init = 1;
	  }
	  if(fabsf(pv_temp - sv_temp)>2.5)
		  pv_reached_sv = 0;
  // ====================================================
	// 2. PV (当前温度/模式) 显示逻辑 - Channel 0
	// ====================================================
	switch (current_state) {
		case STATE_HEAT_READY:
			pv_hold_active = 0;
			pv_reached_sv = 0;
		   // PV 显示 "HEAT"
		   memcpy(pv_data, SEG_HEAT, 4);
		   LED_Green_Set(0); // 准备状态不亮
		   LED_Red_Set(0);
		   break;
		case STATE_AUTOTUNE_READY:
			pv_hold_active = 0;
			pv_reached_sv = 0;
		   // PV 显示 "ATUN"
		   memcpy(pv_data, SEG_ATUN, 4);

		   LED_Green_Set(0);
		   // 红灯闪烁逻辑 (每隔 250ms 切换一次)
		   LED_Red_Set((osKernelGetTickCount() / 250) % 2);
		   break;

		case STATE_HEATING:
		{

		    float abs_err = fabsf(pv_temp - sv_temp);


		    if (!pv_reached_sv)
		    {
		        // 还没到过目标温度
		        if (abs_err < 0.3f)
		        {
		            pv_reached_sv = 1;
		            pv_hold_active = 1;
		            pv_hold = sv_temp;       // 🔒 锁定显示 SV
		            pv_display = pv_hold;
		        }
		        else
		        {
		            // 升温阶段：平滑跟随
		            pv_display += 0.3f * (pv_temp - pv_display);
		        }
		    }
		    else
		    {
		        // 已经到达过：始终显示 SV
		        pv_display = sv_temp;
		    }

		    FloatToSegments(pv_display, pv_data);
		    LED_Green_Set(1);
		    LED_Red_Set(0);
		    break;
		}

		case STATE_AUTOTUNE_RUNNING:

		   // 正常显示当前温度
			pv_reached_sv = 0;
		   FloatToSegments(pv_temp, pv_data);
		   LED_Green_Set(1); // 绿色 LED 常亮
		   LED_Red_Set(0);
		   break;
		case STATE_FAULT:
		   // 显示温度，红灯常亮 (已经在 Temp_Task 中设置)
			pv_reached_sv = 0;
			pv_hold_active = 0;
			pv_display = pv_temp;
			FloatToSegments(pv_display, pv_data);
		   // 可以增加闪烁或显示 "E.rr"
		   // 暂时保持显示温度
		   LED_Green_Set(0);
		   LED_Red_Set(1);
		   break;
		case STATE_OFF:
		default:
		   // 默认：正常显示当前温度
		   pv_hold_active = 0;
		   pv_display = pv_temp;
		   FloatToSegments(pv_display, pv_data);
		   LED_Green_Set(0); // 加热关闭，绿灯灭
		   LED_Red_Set(0);
		   break;
	}


	// 3. 发送 PV 段码数据
	TM1652_SetSegments(TM1652_CHANNEL_PV, pv_data);
	TM1652_SetControl(TM1652_CHANNEL_PV, TM1652_SET_CONTROL_MAX);

// ====================================================
	// 4. SV (目标温度) 显示逻辑 - Channel 1
	// ====================================================
	// MARK: Optimization 2: 实现 SV 调节时的闪烁效果
	if (is_sv_adjusting && current_state == STATE_OFF)
	{
		// 在 SV 调节模式下，SV 值闪烁 (每 250ms 切换一次)
		if ((osKernelGetTickCount() / 250) % 2 == 0) {
			// 显示 SV
			FloatToSegments(sv_temp, sv_data);
		} else {
			// 熄灭 SV
			memcpy(sv_data, SEG_BLANK, 4);
		}
	}
	else
	{
		// 正常显示 SV
		FloatToSegments(sv_temp, sv_data);
	}

	TM1652_SetSegments(TM1652_CHANNEL_SV, sv_data);
	TM1652_SetControl(TM1652_CHANNEL_SV, TM1652_SET_CONTROL_MAX);

  // ====================================================
	// 5. Power/Voltage 显示逻辑 - Channel 2 & 3
	// ====================================================
	Convert_Float_To_Segments_Formatted(power, pow_data,MODE_POWER);
	TM1652_SetSegments(TM1652_CHANNEL_POW, pow_data);
	TM1652_SetControl(TM1652_CHANNEL_POW, TM1652_SET_CONTROL_MID);

	Convert_Float_To_Segments_Formatted(voltage, vol_data,MODE_VOLTAGE);
	TM1652_SetSegments(TM1652_CHANNEL_VOL, vol_data);
	TM1652_SetControl(TM1652_CHANNEL_VOL, TM1652_SET_CONTROL_MID);

	// 6. 精确周期延时
	nextWakeTime += period;
	osDelayUntil(nextWakeTime);
    //osDelay(1);
  }
  /* USER CODE END UI_Task */
}

/* USER CODE BEGIN Header_Key_Task */
/**
* @brief Function implementing the Key thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Key_Task */
void Key_Task(void *argument)
{
  /* USER CODE BEGIN Key_Task */
	const uint32_t period = 50; // 100ms 轮询周期
	uint32_t nextWakeTime = osKernelGetTickCount();
	uint32_t key_states;
	AppState_t current_state;
  /* Infinite loop */
  for(;;)
  {

	// 1. 读取按键状态
	key_states = KEY_Read_State();

  // 2. 获取当前状态（读锁）
  if (osMutexAcquire(xMutexState, osWaitForever) == osOK) {
	  current_state = g_app_state;
	  osMutexRelease(xMutexState);
  }

  if (current_state == STATE_AUTOTUNE_RUNNING) {
      // KEY_Task 完全放弃控制，由 autotune 专用逻辑接管
      nextWakeTime += period;
      osDelayUntil(nextWakeTime);
      continue;
  }

  //  故障和自整定状态下的按键处理
  if (current_state == STATE_FAULT) {
 	  // 故障模式下：短按任意键静音，长按 OK 尝试清除故障
 	  if (Key_Get_ShortPress(KEY_OK) || Key_Get_ShortPress(KEY_MENU)) {
 		  if (osMutexAcquire(xMutexState, 5) == osOK) {
 			  g_fault_muted = 1; // 设置静音标志
 			  osMutexRelease(xMutexState);
 		  }
 		  Beep_Stop(); // 停止当前正在响的蜂鸣器
 		  // 提示音不触发
 	  }

 	  // 长按 OK 尝试清除故障状态（仅在Temp_Task确认故障已解除后才有效）
 	  if (KEY_Get_LongPress(KEY_OK, KEY_MENU_LONG_PRESS_MS)) {
 	      KEY_Reset_LongPress(KEY_OK);
 	      if (osMutexAcquire(xMutexState, 5) == osOK) {
 	          // 只有当故障根源已解除（Temp_Task 会将状态切回 STATE_OFF）时，长按 OK 才有效
 	          if (g_app_state != STATE_FAULT) {
 	              g_app_state = STATE_OFF; // 切换回 OFF 状态
 	              Beep_Start();
 	          }
 	          osMutexRelease(xMutexState);
 	      }
 	  }
 	  // 其他按键在故障模式下被忽略
 	  goto delay_and_continue;
   }
/*  if (current_state == STATE_AUTOTUNE_RUNNING) {
	  // 自整定模式下：短按 Menu 退出自整定
	  if (Key_Get_ShortPress(KEY_MENU)) {
	      if (osMutexAcquire(xMutexState, 5) == osOK) {
	          g_app_state = STATE_OFF; // 退出到 OFF 状态
	          osMutexRelease(xMutexState);
	          g_autotune_request = 0; // 终止自整定任务
	          Beep_Start();
	      }
	  }
	  // 其它按键在自整定运行时被忽略
	  goto delay_and_continue;
  }*/

  // 4. 状态切换逻辑 (Menu/长按Menu)
  if (key_states & KEY_MENU) {
	  // 长按 Menu (5s) -> 尝试进入自整定准备状态
	  if (KEY_Get_LongPress(KEY_MENU, KEY_MENU_LONG_PRESS_MS)) {
		  KEY_Reset_LongPress(KEY_MENU);
		  if (osMutexAcquire(xMutexState, 5) == osOK) {
			  if (g_app_state == STATE_OFF) {
				  g_app_state = STATE_AUTOTUNE_READY;
				  // MARK: Optimization 2: 退出 SV 调节模式
				   g_is_sv_adjusting = 0;
			  }
			  osMutexRelease(xMutexState);
			  Beep_Start();
		  }
	  } else if (Key_Get_ShortPress(KEY_MENU)) {
		  // 短按 Menu
		  if (osMutexAcquire(xMutexState, 5) == osOK) {
			  switch (g_app_state) {
				  case STATE_OFF:
					  // MARK: Optimization 2: 在 OFF 模式下，Menu 短按用于切换 SV 调节模式
					  g_is_sv_adjusting = !g_is_sv_adjusting;
					  break;
				  case STATE_HEAT_READY:
				  case STATE_AUTOTUNE_READY:
					  // 切换回 OFF 状态
					  g_app_state = STATE_OFF;
					  // MARK: Optimization 2: 退出 SV 调节模式
					 g_is_sv_adjusting = 0;
					  break;
				  case STATE_HEATING:
					  // 停止加热
					  g_app_state = STATE_OFF;
					  // MARK: Optimization 2: 退出 SV 调节模式
					  g_is_sv_adjusting = 0;
					  // 提示音
					  Beep_Start();
					  break;
				  default:
					  break;
			  }
			  osMutexRelease(xMutexState);
		  }
		  Beep_Start();
	  }
  }
  // 5. SV 调节逻辑 (只在 STATE_OFF + g_is_sv_adjusting 状态下允许)
  if (current_state == STATE_OFF && g_is_sv_adjusting) {
  	  float step = 1.0f; // 默认步进 1.0
  	  static uint32_t last_repeat_tick = 0;
  	  uint32_t now = osKernelGetTickCount();
  	  if (osMutexAcquire(xMutexState, 5) == osOK) {

	  // ======5. 1. 判断是否进入长按加速模式 ======
  		uint8_t is_long_accel =
			KEY_Get_LongPress(KEY_UP, KEY_ACCEL_PRESS_MS) ||
			KEY_Get_LongPress(KEY_DOWN, KEY_ACCEL_PRESS_MS);

  		  // MARK: Optimization 3: 实现长按加速调节 (10.0度步进)
  		  if (is_long_accel) {
  			  step = ACCEL_STEP_SIZE; // 10.0度
  		  }
  		// ====== 5.2. 短按处理（只触发一次） ======
		if (Key_Get_ShortPress(KEY_UP)) {
			g_setpoint += 1.0f;
			if (g_setpoint > MAX_SETPOINT) g_setpoint = MAX_SETPOINT;
			Beep_Start();
			last_repeat_tick = now;   // 避免短按后立即进入长按重复
		}
		else if (Key_Get_ShortPress(KEY_DOWN)) {
			g_setpoint -= 1.0f;
			if (g_setpoint < MIN_SETPOINT) g_setpoint = MIN_SETPOINT;
			Beep_Start();
			last_repeat_tick = now;
		}
		 // ====== 5.3. 长按自动重复（不使用 key_states & KEY_UP） ======
	if (is_long_accel) {

		// 按住期间每 500ms 增加/减少一次
		if (now - last_repeat_tick >= KEY_ACCEL_PRESS_MS) {
			last_repeat_tick = now;

			if (key_states & KEY_UP) {
				g_setpoint += step;
				if (g_setpoint > MAX_SETPOINT) g_setpoint = MAX_SETPOINT;
				Beep_Start();
			}
			else if (key_states & KEY_DOWN) {
				g_setpoint -= step;
				if (g_setpoint < MIN_SETPOINT) g_setpoint = MIN_SETPOINT;
				Beep_Start();
			}
		}
	}

  		  // 调节完成后，复位加速长按标志，避免下次循环误判
  		  if (!(key_states & KEY_UP) && !(key_states & KEY_DOWN)) {
  			  KEY_Reset_LongPress(KEY_UP);
  			  KEY_Reset_LongPress(KEY_DOWN);
  		  }

  		  osMutexRelease(xMutexState);
  	  }
    }

  // 6. 确认/启动逻辑 (OK Button)
 if (Key_Get_ShortPress(KEY_OK)) {
   if (osMutexAcquire(xMutexState, 5) == osOK) {
	   switch (g_app_state) {
		   case STATE_OFF:
			   // MARK: Optimization 2: 在 OFF 模式且 SV 调节开启时，OK 键用于退出调节模式
			   if (g_is_sv_adjusting) {
				   g_is_sv_adjusting = 0;
				   g_save_pid_request = 1; // SV 设定值已修改，触发保存
				   Beep_Start();
			   }else {
					   // **** 关键修复：不在 SV 调节模式，OK 键进入加热准备模式 ****
					   g_app_state = STATE_HEAT_READY;
					   Beep_Start();
			   }
			   break;
		   case STATE_HEAT_READY:
			   // 启动加热
			   g_app_state = STATE_HEATING;
			   // 启动加热时，保存当前目标温度到 EEPROM
			   g_save_pid_request = 1;
			   Beep_Start();
			   break;
		   case STATE_AUTOTUNE_READY:
			   // 启动自整定（由 Save_Task 负责执行）
			   g_app_state = STATE_AUTOTUNE_RUNNING;
			   g_autotune_request = 1;
			   Beep_Start();
			   break;
		   default:
			   // 其他状态：确认键无效
			   break;
	   }
	   osMutexRelease(xMutexState);
   }
 }
	// 7. 延时
	delay_and_continue:
	nextWakeTime += period;
	osDelayUntil(nextWakeTime);

  }
  /* USER CODE END Key_Task */
}

/* USER CODE BEGIN Header_Save_Task */
/**
* @brief Function implementing the Save thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Save_Task */
void Save_Task(void *argument)
{
  /* USER CODE BEGIN Save_Task */
  /* Infinite loop */
  for(;;)
  {
	  // 1. PID 自整定逻辑 (g_autotune_request 由 Key_Task 设置)
	  if (g_autotune_request)
	  {
		char msg[128];
		snprintf(msg, sizeof(msg), "Starting AutoTune...\r\n");
		osMessageQueuePut(UartQueueHandle, &msg, 0, pdMS_TO_TICKS(10));
		// 确保加热已关闭
		set_heater_impl(0);
		osDelay(pdMS_TO_TICKS(200));// 延迟确保继电器关闭
		PID_Params newparams;
		int ret = PID_AutoTune_SSR(&newparams,
		                                     0.15f,   // step power (+15%)
		                                     0.05f,   // base power (5%)
		                                     30000,   // 60000duration_ms = 60s
		                                     1.0f,    // power_max (100%)
		                                     MAX_SAFE_TEMP_C);
	    // 自整定完成
	    if (osMutexAcquire(xMutexState, osWaitForever) == osOK) {
	        // 无论是成功还是失败，都退出 Autotune 状态，进入 OFF
	        g_app_state = STATE_OFF;
	        osMutexRelease(xMutexState);
	    }

		if (ret == 0)
		{
		  // 保存到 EEPROM
		  // 将参数写入到全局 HeaterPID 并生效
		  if (osMutexAcquire(xMutexPID, osWaitForever) == osOK) {
			  HeaterPID.Kp = newparams.Kp;
			  HeaterPID.Ki = newparams.Ki;
			  HeaterPID.Kd = newparams.Kd;
			  // 重新初始化 PID 内部变量（防止历史积分影响）
			  PID_Init(&HeaterPID, HeaterPID.Kp, HeaterPID.Ki, HeaterPID.Kd, 0.0f, 1.0f);
			  osMutexRelease(xMutexPID);
		  }
		  // 成功后，也需要保存参数和当前的 g_setpoint
		  g_save_pid_request = 1;
		  g_pid_valid = 1;
		  snprintf(msg, sizeof(msg), "AutoTune OK. Kp=%.3f Ki=%.6f Kd=%.3f\r\n",
					newparams.Kp, newparams.Ki, newparams.Kd);
		  osMessageQueuePut(UartQueueHandle, &msg, 0, pdMS_TO_TICKS(10));

		}
		else {
			snprintf(msg, sizeof(msg), "AutoTune ERR: %d. Params NOT SAVED.\r\n", ret);
			 osMessageQueuePut(UartQueueHandle, &msg, 0, pdMS_TO_TICKS(10));
			// 失败时保持 g_pid_valid 不变
			  }
		  // 清除请求
		   g_autotune_request = 0;
	  }
	  // 2. PID/SV 参数保存逻辑 (g_save_pid_request 由 Key_Task 或 Autotune 成功设置)
	  if (g_save_pid_request) {

		   PID_Save_t save;

		   // 锁定 PID 和状态，确保获取到最新的 K/SV 值
		   if (osMutexAcquire(xMutexPID, osWaitForever) == osOK && osMutexAcquire(xMutexState, osWaitForever) == osOK) {
			   save.Kp = HeaterPID.Kp;
			   save.Ki = HeaterPID.Ki;
			   save.Kd = HeaterPID.Kd;
			   save.Setpoint = g_setpoint; // 保存目标温度
			   save.magic = PID_MAGIC;
			   osMutexRelease(xMutexState);
			   osMutexRelease(xMutexPID);
		   } else {
			   // 获取锁失败，本次跳过保存，等待下一周期
			   goto delay_and_continue;
		   }

		   // 写入 EEPROM（逐字节写）
			 uint8_t *p = (uint8_t*)&save;
			 for (uint32_t i=0; i<sizeof(PID_Save_t); i++) {
				 AT24_WriteByte(EEPROM_PID_ADDR + i, p[i]);
				// osDelay(pdMS_TO_TICKS(6)); // 写周期保护
			 }

			 g_save_pid_request = 0; // 清除保存请求
			 char msg[64];
			 snprintf(msg, sizeof(msg), "PID/SV parameters saved to EEPROM.\r\n");
			 osMessageQueuePut(UartQueueHandle, &msg, 0, pdMS_TO_TICKS(10));
	  }
	  delay_and_continue:
	  osDelay(pdMS_TO_TICKS(200));
   // osDelay(1);
  }
  /* USER CODE END Save_Task */
}

/* USER CODE BEGIN Header_Beep_Task */
/**
* @brief Function implementing the Beep thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Beep_Task */
void Beep_Task(void *argument)
{
  /* USER CODE BEGIN Beep_Task */
  /* Infinite loop */
  for(;;)
  {
	  // 阻塞等待 Beep 信号量的通知
	      if(osSemaphoreAcquire(beep_signal_sem, osWaitForever) == osOK)
	      {
	          // 开启蜂鸣器 (PA15 高电平叫)
	          HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, GPIO_PIN_SET);

	          // 持续 100ms
	          osDelay(pdMS_TO_TICKS(100));

	          // 关闭蜂鸣器 (调用 bsp_io.c 中的 Beep_Stop，该函数会将 PA15 拉低)
	          Beep_Stop();
	      }
  }
  /* USER CODE END Beep_Task */
}

/* USER CODE BEGIN Header_UART_Print_Task */
/**
* @brief Function implementing the UART_Print thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Print_Task */
void UART_Print_Task(void *argument)
{
  /* USER CODE BEGIN UART_Print_Task */
	 char msg[128];
  /* Infinite loop */
  for(;;)
  {
	  // 等待消息
	 if(osMessageQueueGet(UartQueueHandle, &msg, NULL, osWaitForever) == osOK)
	 {
		 // 阻塞方式发送，可以改为 DMA
		 HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	 }
	  //osDelay(1);
  }
  /* USER CODE END UART_Print_Task */
}

/* USER CODE BEGIN Header_Fault_detector_Task */
/**
* @brief Function implementing the Fault_detector thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Fault_detector_Task */
void Fault_detector_Task(void *argument)
{
  /* USER CODE BEGIN Fault_detector_Task */
uint32_t nextWakeTime = osKernelGetTickCount();
	const uint32_t period = 500; // 500ms 检测周期

	float last_temp = 0.0f;
	uint32_t full_power_count = 0;
	const uint32_t FULL_POWER_LIMIT = 10; // 10次 * 0.5s = 10s
	const float TEMP_RISE_THRESHOLD = 1.0f;   // 温度未上升阈值
	const float TEMP_DROP_THRESHOLD = 5.0f;   // 温度快速下降阈值 (°C/周期)
	uint16_t overpower_cnt = 0;
	HLW8032_Value_t power_data;
	AppState_t current_state;
/* Infinite loop */
  for(;;)
  {
	  // 读取当前状态
	if(osMutexAcquire(xMutexState, osWaitForever) == osOK) {
		current_state = g_app_state;
		osMutexRelease(xMutexState);
	}

	// 只在加热相关状态检测
	if(current_state == STATE_HEATING || current_state == STATE_HEAT_READY)
	{
		// 读取温度
		float temp = 0.0f;
		if(osMutexAcquire(xMutexTemp, osWaitForever) == osOK) {
			temp = PID_Input_Temperature;
			osMutexRelease(xMutexTemp);
		}

		// 读取功率
		if(osMutexAcquire(xMutexPower, osWaitForever) == osOK) {
			power_data = HLW8032_GetData();
			osMutexRelease(xMutexPower);
		}
		float power = power_data.Power;

		// =============================
		// 1. 功率过大判断 (2500W)
		// =============================
		if(power > 2500)
		{
			overpower_cnt++;
			if(overpower_cnt > 2) {   // 1 秒
				if(osMutexAcquire(xMutexState, osWaitForever) == osOK)
				{
					g_app_state = STATE_FAULT;
					g_fault_muted = 0;
					osMutexRelease(xMutexState);
				}
				set_heater_impl(0);
				osSemaphoreRelease(beep_signal_sem);
				overpower_cnt = 0;
			}
		} else
			{
				overpower_cnt = 0;
			}
		// -------------------------
		// 加热棒异常检测（功率高但温度不上升）
		// -------------------------
		if(power > 100) {  //定义功率大于100W  连续输出功率>100W 但不升温（恒温时会进行控温）
			if(fabsf(temp - last_temp) < TEMP_RISE_THRESHOLD) {
				full_power_count++;
			} else {
				full_power_count = 0;
			}

			if(full_power_count >= FULL_POWER_LIMIT ) {
				// 故障判定
				if(osMutexAcquire(xMutexState, osWaitForever) == osOK) {
					g_app_state = STATE_FAULT;
					g_fault_muted = 0;
					osMutexRelease(xMutexState);
				}
				set_heater_impl(0);
				osSemaphoreRelease(beep_signal_sem);
				full_power_count = 0;
			}
		} else {
			full_power_count = 0;
		}

		// -------------------------
		// 温度快速下降异常检测
		// -------------------------
		float delta_temp = temp - last_temp;
		if(delta_temp < -TEMP_DROP_THRESHOLD) {
			// 温度快速下降 -> 传感器脱离或异常
			if(osMutexAcquire(xMutexState, osWaitForever) == osOK) {
				g_app_state = STATE_FAULT;
				g_fault_muted = 0;
				osMutexRelease(xMutexState);
			}
			set_heater_impl(0);
			osSemaphoreRelease(beep_signal_sem);
		}

		last_temp = temp;
	}

	nextWakeTime += period;
	osDelayUntil(nextWakeTime);
  }
  /* USER CODE END Fault_detector_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

float Temp_Filter(float newVal)
{
    tempBuffer[tempIndex] = newVal;
    tempIndex = (tempIndex + 1) % AVG_N;

    if (tempCount < AVG_N)
        tempCount++;

    float sum = 0.0f;
    for (int i = 0; i < tempCount; i++)
        sum += tempBuffer[i];

    return sum / tempCount;
}

/**
 * @brief 将浮点数转换为 TM1652 的 4 位段码显示（支持小数点控制）
 *
 * @param value       输入浮点数
 * @param out_seg     输出段码 (4 bytes)
 * @param mode        显示模式 (功率/电压)
 *
 * MODE_POWER 规则：
 *   <1000: 保留一位小数 (显示 XXX.X)
 *   >=1000: 显示整数，不带小数点
 *
 * MODE_VOLTAGE 规则：
 *   始终保留一位小数 (XX.X 或 XXX.X)
 */
void Convert_Float_To_Segments_Formatted(float value, uint8_t out_seg[4], DisplayMode_t mode)
{
    uint8_t seg_blank = TM1652_SegmentMap[11];  // Blank
    uint8_t seg_minus = TM1652_SegmentMap[10];  // '-'



    // 限制负数（功率/电压不会为负）
    if (value < 0) value = 0;

    float abs_val = value;

    uint16_t scaled = 0;
    uint8_t use_decimal = 0;

    // ================================
    // 1. 功率模式
    // ================================
    if (mode == MODE_POWER) {
        if (abs_val < 1000.0f) {
            use_decimal = 1;  // <1000W 时保留小数
            scaled = (uint16_t)(abs_val * 10 + 0.5f);  // x.x
        } else {
            use_decimal = 0;  // >=1000W 不带小数
            scaled = (uint16_t)(abs_val + 0.5f);
        }
        // ---- 超出范围保护 ----
        if (scaled > 9999) {
            out_seg[0] = seg_minus;
            out_seg[1] = seg_minus;
            out_seg[2] = seg_minus;
            out_seg[3] = seg_minus;
            return;
        }
    }

    // ================================
    // 2. 电压模式：永远保留 1 位小数
    // ================================
    else if (mode == MODE_VOLTAGE) {
        use_decimal = 1;
        scaled = (uint16_t)(abs_val * 10 + 0.5f);  // x.x
    }

    // 避免显示溢出
    if (scaled > 9999)
        scaled = 9999;

    // 分解四位
    uint8_t d0 = (scaled / 1000) % 10;   // 千
    uint8_t d1 = (scaled / 100)  % 10;   // 百
    uint8_t d2 = (scaled / 10)   % 10;   // 十
    uint8_t d3 = (scaled      )  % 10;   // 个

    // 写段码
    out_seg[0] = TM1652_SegmentMap[d0];
    out_seg[1] = TM1652_SegmentMap[d1];
    out_seg[2] = TM1652_SegmentMap[d2];
    out_seg[3] = TM1652_SegmentMap[d3];

    // 加小数点
    if (use_decimal) {
        out_seg[2] |= SEG_DP_MASK;   // X X X . X（个位右侧有小数点）
    }

    // 前导零消隐（不要消隐 d2，因为那里有小数点）
    if (d0 == 0) out_seg[0] = seg_blank;
    if (d0 == 0 && d1 == 0) out_seg[1] = seg_blank;

    // 对于 <10 时，使显示更美观
    if (use_decimal && scaled < 100) {
        out_seg[0] = seg_blank;
        out_seg[1] = seg_blank;
    }
}
/**
 * @brief 将浮点温度转换为4位数码管的段码数据（支持负数）
 * @param temp: 浮点温度值 (e.g., 123.4, -12.3)
 * @param output: 4字节段码输出数组 (DDR0-DDR3)
 * @note 格式化为 X X X . X (保留一位小数)。
 * @note 正数范围: 0.0 到 999.9。负数范围: -99.9 到 0.0。
 */
static void FloatToSegments(float temp, uint8_t output[4])
{
    uint8_t is_negative = (temp < 0.0f);
    float abs_temp = fabsf(temp);

    // 1. 处理范围限制
    // 正数最大 999.9。负数最大 -99.9 (因为 DDR0 需要显示负号 '-')
    if (abs_temp > 999.9f || (is_negative && abs_temp > 99.9f)) {
        // 超过范围，显示 ----
        output[0] = TM1652_SegmentMap[10]; // 10: 减号 '-'
        output[1] = TM1652_SegmentMap[10];
        output[2] = TM1652_SegmentMap[10];
        output[3] = TM1652_SegmentMap[10];
        return;
    }

    // 2. 放大10倍并取整，以便提取每一位数字 (e.g., 12.3 -> 123)
    uint16_t value = (uint16_t)(roundf(abs_temp * 10.0f));

    // ====================================================
    // 3. 提取数字和转换段码 (DDR0 DDR1 DDR2. DDR3)
    // ====================================================

    uint8_t d3 = value % 10;          // 小数点后一位
    uint8_t d2 = (value / 10) % 10;   // 个位 (带DP)

    output[3] = TM1652_SegmentMap[d3];
    // 个位带小数点
    output[2] = TM1652_SegmentMap[d2] | SEG_DP_MASK;

    if (is_negative) {
        // 负数显示: - X X . X (DDR0 = -, DDR1 = 十位, DDR2 = 个位, DDR3 = 小数位)
        uint8_t d1 = (value / 100) % 10;  // 十位

        output[0] = TM1652_SegmentMap[10]; // DDR0 显示负号 '-'
        output[1] = TM1652_SegmentMap[d1];

        // 前导零消隐 (只需对 DDR1，即十位进行消隐, 如果值小于10.0，如 -5.2)
        // value < 100 意味着绝对值小于 10.0 (e.g., 5.2 -> value=52)
        if (value < 100) {
            output[1] = TM1652_SegmentMap[11]; // 11: Blank
        }

    } else {
        // 正数显示: X X X . X (DDR0 = 百位, DDR1 = 十位, DDR2 = 个位, DDR3 = 小数位)
        uint8_t d1 = (value / 100) % 10;  // 十位
        uint8_t d0 = (value / 1000) % 10; // 百位

        output[1] = TM1652_SegmentMap[d1];
        output[0] = TM1652_SegmentMap[d0];

        // 前导零消隐 (DDR0 百位, DDR1 十位)
        // 假设 SegmentMap[11] 为消隐 (Blank)
        if (d0 == 0) {
            output[0] = TM1652_SegmentMap[11];
            if (d1 == 0) {
                output[1] = TM1652_SegmentMap[11];
                // 此时剩下 X. X，个位不能消隐
            }
        }
    }
}

/* 从 EEPROM 读取 PID 参数（逐字节读取） */
static int PID_LoadFromEEPROM(void)
{
    PID_Save_t load;
    uint8_t *p = (uint8_t*)&load;

    for (uint32_t i=0; i<sizeof(PID_Save_t); i++) {
        p[i] = AT24_ReadByte(EEPROM_PID_ADDR + i);
        delay_us(1000);
      //  HAL_Delay(1);
     //   osDelay(1);
    }

    if (load.magic != PID_MAGIC) {
        return -1; // 无效
    }

    // 将值复制到 HeaterPID（线程安全在调用处处理）
    HeaterPID.Kp = load.Kp;
    HeaterPID.Ki = load.Ki;
    HeaterPID.Kd = load.Kd;
    g_setpoint = load.Setpoint; // 加载目标温度
    return 0;
}
/* 读取当前温度（用于 autotune） */
 float read_temp_impl(void)
{
    float t = 0.0f;
    if (osMutexAcquire(xMutexTemp, osWaitForever) == osOK) {
        t = PID_Input_Temperature;
        osMutexRelease(xMutexTemp);
    }
    return t;
}
/* 设置加热功率（0..1），映射到定时器比较值 */
 void set_heater_impl(uint8_t on)
{
	if(on)
	{
		Heat_on;
	}
	else
	{
		Heat_off;
	}


}
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
	 HLW8032_RxCplt(huart);

 }
/* USER CODE END Application */

