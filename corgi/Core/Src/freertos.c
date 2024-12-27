/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fsm.h"
#include "walk.h"
#include "Bluetooth.h"
#include "action.h"
#include "jump.h"
#include "dog.h"
#include "obstacle_course.h"
#include "pos_ctrl.h"
#include "speed_race.h"

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

/* USER CODE END Variables */
osThreadId Spd_RaceHandle;
osThreadId GamepadHandle;
osThreadId debugHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Spd_Race_Ctl(void const * argument);
void Gamepad_Ctl(void const * argument);
void debug_fun(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Spd_Race */
  osThreadDef(Spd_Race, Spd_Race_Ctl, osPriorityNormal, 0, 128);
  Spd_RaceHandle = osThreadCreate(osThread(Spd_Race), NULL);

  /* definition and creation of Gamepad */
  osThreadDef(Gamepad, Gamepad_Ctl, osPriorityIdle, 0, 128);
  GamepadHandle = osThreadCreate(osThread(Gamepad), NULL);

  /* definition and creation of debug */
  osThreadDef(debug, debug_fun, osPriorityIdle, 0, 128);
  debugHandle = osThreadCreate(osThread(debug), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Spd_Race_Ctl */

int sign_stop = 0;
void start_race(void);
int start_race_flag = 0;
int linkstart = 0;
uint8_t send2vis[8] = {0x0A,0x0E,0x00,0x00,0x00,0x00,0x00,0x05};
extern int16_t g_sbus_channels[8];
extern speed_race_param Auto_spd;
/**
  * @brief  Function implementing the Spd_Race thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Spd_Race_Ctl */
void Spd_Race_Ctl(void const * argument)
{
  /* USER CODE BEGIN Spd_Race_Ctl */
  /* Infinite loop */
  for(;;)
  {
		if (start_race_flag == -2)
		{
			SetDogChangeAble(ENABLE);
			ChangeDogState(Keep);
			Change_Ctrl_State(Pos_Ctrl);
		}
		
		if ((start_race_flag == 1) || (start_race_flag == 2))
		{
			start_race();
			Change_Ctrl_State(Pos_Ctrl);
			Change_Dog_Pitch_State(Follow_Ground);
		}
		if (start_race_flag == -1)
		{
			SetDogChangeAble(ENABLE);
			ChangeDogState(Keep);
			Change_Ctrl_State(Force_Ctrl);
		}
	

//		if ((g_sbus_channels[7] == 192) && (linkstart == 0))
//		{
//			SetDogChangeAble(ENABLE);
//			ChangeDogState(Keep);
//			Change_Ctrl_State(Force_Ctrl);
//			linkstart = 1;
//			
//		}
//		else if ((g_sbus_channels[7] == 992) && (linkstart == 1))
//		{
//			ChangeDogState(Stand);
//			SetDogChangeAble(ENABLE);
//			ChangeDogState(Keep);
//			Change_Ctrl_State(Pos_Ctrl);
//			linkstart = 2;
//		}
//		else if ((g_sbus_channels[7] == 1792) && (linkstart == 2))
//		{
//			Auto_spd.start_race_flag = 1;
//			Change_Ctrl_State(Pos_Ctrl);
//			Change_Dog_Pitch_State(Follow_Ground);
//			for(int i = 0;i<=100;i++)
//			if(HAL_UART_Transmit(&huart3, &send2vis[0], 8, 0xFFF) == HAL_OK)
//			linkstart = 3;
//		}
//		else if ((g_sbus_channels[7] == 1792) && (linkstart == 3))
//		{
//				start_race();
//		}
		osDelay(2);
//		if(HAL_UART_Transmit(&huart3, &send2vis[0], 8, 0xFFF) == HAL_OK);
  }
  /* USER CODE END Spd_Race_Ctl */
}

/* USER CODE BEGIN Header_Gamepad_Ctl */
/**
* @brief Function implementing the Gamepad thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gamepad_Ctl */
void Gamepad_Ctl(void const * argument)
{
  /* USER CODE BEGIN Gamepad_Ctl */
  /* Infinite loop */
  for(;;)
  {
		GamePad_Control_Obs();
//		GamePad_Control_Country();
    osDelay(1);
  }
  /* USER CODE END Gamepad_Ctl */
}

/* USER CODE BEGIN Header_debug_fun */



int walk_able_flag = 0;
float Pos_x = 0;
float Pos_y = 0;
void GamePad_Control_Action(void)
{
  //  Action参数
  Action_Param.VMC_Swing_Stand[0].Kp_x = Rx.Short_data[0];
  Action_Param.VMC_Swing_Stand[0].Kp_z = Rx.Short_data[1];
  Action_Param.VMC_Swing_Stand[0].Kd_x = Rx.Short_data[2];
  Action_Param.VMC_Swing_Stand[0].Kd_z = Rx.Short_data[3];

  Action_Param.VMC_Swing_Stand[1].Kp_x = Rx.Short_data[4];
  Action_Param.VMC_Swing_Stand[1].Kp_z = Rx.Short_data[5];
  Action_Param.VMC_Swing_Stand[1].Kd_x = Rx.Short_data[6];
  Action_Param.VMC_Swing_Stand[1].Kd_z = Rx.Short_data[7];

  Action_Param.VMC_Swing_Stand[2].Kp_x = Rx.Short_data[8];
  Action_Param.VMC_Swing_Stand[2].Kp_z = Rx.Short_data[9];
  Action_Param.VMC_Swing_Stand[2].Kd_x = Rx.Short_data[10];
  Action_Param.VMC_Swing_Stand[2].Kd_z = Rx.Short_data[11];

  Action_Param.VMC_Swing_Stand[3].Kp_x = Rx.Short_data[12];
  Action_Param.VMC_Swing_Stand[3].Kp_z = Rx.Short_data[13];
  Action_Param.VMC_Swing_Stand[3].Kd_x = Rx.Short_data[14];
  Action_Param.VMC_Swing_Stand[3].Kd_z = Rx.Short_data[15];

  sign_stop = Rx.Short_data[16];
  for (int i = 0; i < 4; i++)
  {
    Action_Param.VMC_Stance_Keep[i].Kp_x = Action_Param.VMC_Swing_Stand[i].Kp_x;
    Action_Param.VMC_Stance_Keep[i].Kd_x = Action_Param.VMC_Swing_Stand[i].Kd_x;
    Action_Param.VMC_Stance_Keep[i].Kp_z = Action_Param.VMC_Swing_Stand[i].Kp_z;
    Action_Param.VMC_Stance_Keep[i].Kd_z = Action_Param.VMC_Swing_Stand[i].Kd_z;
    Action_Param.VMC_Swing_Squat[i].Kp_x = Action_Param.VMC_Swing_Stand[i].Kp_x;
    Action_Param.VMC_Swing_Squat[i].Kd_x = Action_Param.VMC_Swing_Stand[i].Kd_x;
    Action_Param.VMC_Swing_Squat[i].Kp_z = Action_Param.VMC_Swing_Stand[i].Kp_z;
    Action_Param.VMC_Swing_Squat[i].Kd_z = Action_Param.VMC_Swing_Stand[i].Kd_z;
  }
}

void GamePad_Control_Gait(void)
{
  // gait参数
  gait_param.VMC_Swing_Walk[0].Kp_x = Rx.Short_data[0];
  gait_param.VMC_Swing_Walk[0].Kd_x = Rx.Short_data[1];
  gait_param.VMC_Swing_Walk[0].Kp_z = Rx.Short_data[2];
  gait_param.VMC_Swing_Walk[0].Kd_z = Rx.Short_data[3];

  gait_param.VMC_Swing_Walk[1].Kp_x = Rx.Short_data[4];
  gait_param.VMC_Swing_Walk[1].Kd_x = Rx.Short_data[5];
  gait_param.VMC_Swing_Walk[1].Kp_z = Rx.Short_data[6];
  gait_param.VMC_Swing_Walk[1].Kd_z = Rx.Short_data[7];

  gait_param.VMC_Swing_Walk[2].Kp_x = Rx.Short_data[8];
  gait_param.VMC_Swing_Walk[2].Kd_x = Rx.Short_data[9];
  gait_param.VMC_Swing_Walk[2].Kp_z = Rx.Short_data[10];
  gait_param.VMC_Swing_Walk[2].Kd_z = Rx.Short_data[11];

  gait_param.VMC_Swing_Walk[3].Kp_x = Rx.Short_data[12];
  gait_param.VMC_Swing_Walk[3].Kd_x = Rx.Short_data[13];
  gait_param.VMC_Swing_Walk[3].Kp_z = Rx.Short_data[14];
  gait_param.VMC_Swing_Walk[3].Kd_z = Rx.Short_data[15];

  gait_param.VMC_Stance_Walk[0].Kp_x = Rx.Short_data[16];
  gait_param.VMC_Stance_Walk[0].Kd_x = Rx.Short_data[17];
  gait_param.VMC_Stance_Walk[0].Kp_z = Rx.Short_data[18];
  gait_param.VMC_Stance_Walk[0].Kd_z = Rx.Short_data[19];

  gait_param.VMC_Stance_Walk[1].Kp_x = Rx.Short_data[20];
  gait_param.VMC_Stance_Walk[1].Kd_x = Rx.Short_data[21];
  gait_param.VMC_Stance_Walk[1].Kp_z = Rx.Short_data[22];
  gait_param.VMC_Stance_Walk[1].Kd_z = Rx.Short_data[23];

  gait_param.VMC_Stance_Walk[2].Kp_x = Rx.Short_data[24];
  gait_param.VMC_Stance_Walk[2].Kd_x = Rx.Short_data[25];
  gait_param.VMC_Stance_Walk[2].Kp_z = Rx.Short_data[26];
  gait_param.VMC_Stance_Walk[2].Kd_z = Rx.Short_data[27];

  gait_param.VMC_Stance_Walk[3].Kp_x = Rx.Short_data[28];
  gait_param.VMC_Stance_Walk[3].Kd_x = Rx.Short_data[29];
  gait_param.VMC_Stance_Walk[3].Kp_z = Rx.Short_data[30];
  gait_param.VMC_Stance_Walk[3].Kd_z = Rx.Short_data[31];
  sign_stop = Rx.Short_data[32];

  //  walk_able_flag = Rx.Short_data[33];
  gait_param.Dire = Rx.Short_data[35];
  gait_param.VMC_Stance_Walk[0].Feedforward_z = Rx.Short_data[36];
  gait_param.VMC_Stance_Walk[1].Feedforward_z = Rx.Short_data[37];
  gait_param.VMC_Stance_Walk[2].Feedforward_z = Rx.Short_data[38];
  gait_param.VMC_Stance_Walk[3].Feedforward_z = Rx.Short_data[39];
  leg[0].ctrl_state = Rx.Short_data[40];

  gait_param.std_step_length = Rx.Float_data[0];
  gait_param.term = Rx.Float_data[1];
  gait_param.step_height = Rx.Float_data[2];
  gait_param.step_height_down = Rx.Float_data[3];
  Pos_x = Rx.Float_data[4];
  Pos_y = Rx.Float_data[5];

  if (sqrtf(Pos_x * Pos_x + Pos_y * Pos_y) < 300)
  {
    Euler_Angle.Gamepad_Last_Yaw = 0;
    Euler_Angle.Target_Yaw = 0;
    gait_param.std_step_length = 0;
  }
  else
  {
    float Dleta_Angle1, Dleta_Angle2;
    float now_ctrl_yaw = atan2f(-Pos_x, Pos_y) * 57.296f;
    if (fabs(now_ctrl_yaw) <= 15)
      now_ctrl_yaw = 0.00001;
    if (Euler_Angle.Gamepad_Last_Yaw == 0)
    {
      Euler_Angle.Target_Yaw += now_ctrl_yaw;
      Euler_Angle.Gamepad_Last_Yaw = now_ctrl_yaw;
    }

    if (now_ctrl_yaw > (Euler_Angle.Gamepad_Last_Yaw))
    {
      Dleta_Angle1 = now_ctrl_yaw - Euler_Angle.Gamepad_Last_Yaw;
      Dleta_Angle2 = -360.0f + now_ctrl_yaw - Euler_Angle.Gamepad_Last_Yaw;
    }
    else
    {
      Dleta_Angle1 = now_ctrl_yaw - Euler_Angle.Gamepad_Last_Yaw;
      Dleta_Angle2 = 360.0f + now_ctrl_yaw - Euler_Angle.Gamepad_Last_Yaw;
    }

    if ((fabs(Dleta_Angle1)) > (fabs(Dleta_Angle2)))
    {
      Euler_Angle.Target_Yaw += Dleta_Angle2;
    }
    else
    {
      Euler_Angle.Target_Yaw += Dleta_Angle1;
    }
    Euler_Angle.Gamepad_Last_Yaw = now_ctrl_yaw;
  }
}

void GamePad_Control_Jump(void)
{
  // 	jump参数
  extern Jump_VMC_Param Jump_VMC;

  Jump_VMC.VMC_Swing[0].Kp_x = Rx.Short_data[0];
  Jump_VMC.VMC_Swing[0].Kd_x = Rx.Short_data[1];
  Jump_VMC.VMC_Swing[0].Kp_z = Rx.Short_data[2];
  Jump_VMC.VMC_Swing[0].Kd_z = Rx.Short_data[3];

  Jump_VMC.VMC_Swing[1].Kp_x = Rx.Short_data[4];
  Jump_VMC.VMC_Swing[1].Kd_x = Rx.Short_data[5];
  Jump_VMC.VMC_Swing[1].Kp_z = Rx.Short_data[6];
  Jump_VMC.VMC_Swing[1].Kd_z = Rx.Short_data[7];

  Jump_VMC.VMC_Swing[2].Kp_x = Rx.Short_data[8];
  Jump_VMC.VMC_Swing[2].Kd_x = Rx.Short_data[9];
  Jump_VMC.VMC_Swing[2].Kp_z = Rx.Short_data[10];
  Jump_VMC.VMC_Swing[2].Kd_z = Rx.Short_data[11];

  Jump_VMC.VMC_Swing[3].Kp_x = Rx.Short_data[12];
  Jump_VMC.VMC_Swing[3].Kd_x = Rx.Short_data[13];
  Jump_VMC.VMC_Swing[3].Kp_z = Rx.Short_data[14];
  Jump_VMC.VMC_Swing[3].Kd_z = Rx.Short_data[15];

  Jump_VMC.VMC_Cushion[0].Kp_x = Rx.Short_data[16];
  Jump_VMC.VMC_Cushion[0].Kd_x = Rx.Short_data[17];
  Jump_VMC.VMC_Cushion[0].Kp_z = Rx.Short_data[18];
  Jump_VMC.VMC_Cushion[0].Kd_z = Rx.Short_data[19];

  Jump_VMC.VMC_Cushion[1].Kp_x = Rx.Short_data[20];
  Jump_VMC.VMC_Cushion[1].Kd_x = Rx.Short_data[21];
  Jump_VMC.VMC_Cushion[1].Kp_z = Rx.Short_data[22];
  Jump_VMC.VMC_Cushion[1].Kd_z = Rx.Short_data[23];

  Jump_VMC.VMC_Cushion[2].Kp_x = Rx.Short_data[24];
  Jump_VMC.VMC_Cushion[2].Kd_x = Rx.Short_data[25];
  Jump_VMC.VMC_Cushion[2].Kp_z = Rx.Short_data[26];
  Jump_VMC.VMC_Cushion[2].Kd_z = Rx.Short_data[27];

  Jump_VMC.VMC_Cushion[3].Kp_x = Rx.Short_data[28];
  Jump_VMC.VMC_Cushion[3].Kd_x = Rx.Short_data[29];
  Jump_VMC.VMC_Cushion[3].Kp_z = Rx.Short_data[30];
  Jump_VMC.VMC_Cushion[3].Kd_z = Rx.Short_data[31];
	
  sign_stop = Rx.Short_data[32];

  extern JumpType Jumptype;
  extern JumpParam Jump_Param[15];
  for (int i = 0; i <= 3; i++)
  {
    Jump_Param[Jumptype].Prep_Rho[i] = Rx.Float_data[0];
    Jump_Param[Jumptype].Jump_Time[i] = Rx.Float_data[3];
    Jump_Param[Jumptype].Tuck_Rho[i] = Rx.Float_data[4];
    Jump_Param[Jumptype].Load_Rho[i] = Rx.Float_data[6];
    Jump_Param[Jumptype].Tuck_Time[i] = Rx.Float_data[8];
  }
//  for (int i = 0; i <= 3; i++)
//  {
//    Jump_Param[Jumptype].Std_Prep_Theta = Rx.Float_data[1];
//    Jump_Param[Jumptype].Std_Jump_Theta = Rx.Float_data[2];
//    Jump_Param[Jumptype].Std_Tuck_Theta = Rx.Float_data[5];
//    Jump_Param[Jumptype].Std_Load_Theta = Rx.Float_data[7];
//  }
}

void GamePad_Control_Block(void)
{
  leg[0].ctrl_state = Rx.Short_data[0];
	
  if ((Rx.Short_data[1] == 1) && (Rx.Short_data[2] == 0) && (Rx.Short_data[3] == 0))
  {
    sign_stop = 4;
  }
  else if ((Rx.Short_data[1] == 0) && (Rx.Short_data[2] == 1) && (Rx.Short_data[3] == 0) && (leg[0].ctrl_state == Force_Ctrl))
  {
    sign_stop = 1;
  }
	else if ((Rx.Short_data[1] == 0) && (Rx.Short_data[2] == 0) && (Rx.Short_data[3] == 1) && (leg[0].ctrl_state == Force_Ctrl))
  {
    sign_stop = 1;
  }
  else if ((Rx.Short_data[1] == 0) && (Rx.Short_data[2] == 0) && (Rx.Short_data[3] == 0))
  {
    sign_stop = 1;
  }
  gait_param.std_step_length = Rx.Float_data[0];
  gait_param.term = Rx.Float_data[1];
  gait_param.step_height = Rx.Float_data[2];
  gait_param.step_height_down = Rx.Float_data[3];

  Pos_x = Rx.Float_data[4];
  Pos_y = Rx.Float_data[5];

  if (sqrtf(Pos_x * Pos_x + Pos_y * Pos_y) < 300.0f)
  {
    Euler_Angle.Gamepad_Last_Yaw = 0;
    Euler_Angle.Target_Yaw = 0;
    gait_param.std_step_length = 0;
  }
  else
  {
    float Dleta_Angle1, Dleta_Angle2;
    float now_ctrl_yaw = atan2f(-Pos_x, Pos_y) * 57.296f;
    if (fabs(now_ctrl_yaw) <= 10)
      now_ctrl_yaw = 0.00001;
    if (Euler_Angle.Gamepad_Last_Yaw == 0)
    {
      Euler_Angle.Target_Yaw += now_ctrl_yaw;
      Euler_Angle.Gamepad_Last_Yaw = now_ctrl_yaw;
    }

    if (now_ctrl_yaw > (Euler_Angle.Gamepad_Last_Yaw))
    {
      Dleta_Angle1 = now_ctrl_yaw - Euler_Angle.Gamepad_Last_Yaw;
      Dleta_Angle2 = -360.0f + now_ctrl_yaw - Euler_Angle.Gamepad_Last_Yaw;
    }
    else
    {
      Dleta_Angle1 = now_ctrl_yaw - Euler_Angle.Gamepad_Last_Yaw;
      Dleta_Angle2 = 360.0f + now_ctrl_yaw - Euler_Angle.Gamepad_Last_Yaw;
    }

    if ((fabs(Dleta_Angle1)) > (fabs(Dleta_Angle2)))
    {
      Euler_Angle.Target_Yaw += Dleta_Angle2;
    }
    else
    {
      Euler_Angle.Target_Yaw += Dleta_Angle1;
    }
    Euler_Angle.Gamepad_Last_Yaw = now_ctrl_yaw;
  }
}


/**
* @brief Function implementing the debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_debug_fun */
void debug_fun(void const * argument)
{
  /* USER CODE BEGIN debug_fun */
  /* Infinite loop */
  for(;;)
  {
		osDelay(20);

// 启动蓝牙的时候记得把定时器里面防断连打开
// 记得改bluetooth.h
//    GamePad_Control_Gait();
//    GamePad_Control_Jump(); //我现在把调整pd参数给关了，调的时候记得打开
//		GamePad_Control_Action();
//		GamePad_Control_Block();

    if (sign_stop == 1)
    {
      if (GetNowDogState() != Keep)
      {
        ChangeDogState(Keep);
      }
    }
    else if (sign_stop == 2)
    {
      if (GetNowDogState() != Squat)
      {
        ChangeDogState(Squat);
      }
    }
    else if (sign_stop == 3)
    {
      if (GetNowDogState() != Stand)
      {
        ChangeDogState(Stand);
      }
    }
    else if (sign_stop == 4)
    {
      if (GetNowDogState() != Walk)
      {
        ChangeDogState(Walk);
      }
    }
    else if (sign_stop == 5)
    {
      if (GetNowDogState() != Jump)
      {
        ChangeDogState(Jump);
      }
    }
    else if (sign_stop == 6)
    {
      if (GetNowDogState() != Backflip)
      {
        ChangeDogState(Backflip);
      }
    }
		else if (sign_stop == 7)
    {
      if (GetNowDogState() != Start)
      {
        ChangeDogState(Start);
      }
    }
  }  
  /* USER CODE END debug_fun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
