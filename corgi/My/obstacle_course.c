#include "pos_ctrl.h"
#include "dog.h"
#include "walk.h"
#include "math.h"
#include "stm32h7xx.h"
#include "sbus.h"
#include "walk.h"
#include "fsm.h"
#include "obstacle_course.h"
#include "Backflip.h"

/*
右摇杆 前后左右
左摇杆:
					无
		跳跃      后空翻
					无
后空翻：
在双木桥上 左腿对齐双木桥中间横杆的正中间
靠最右面原地起跳，第一次第二次都是
台阶 靠最右边 腿最长关节处和楼梯差不多重合

跳跃：
GroundForward在楼梯上往上跳 脚在楼梯中间 身体靠左防止掉下去
 下台阶：步高0.02 周期0.8 下沉步高0 步长0.08 偏航修正Kp0.015 最大修正步长0.06 力控 加垂直于地面修正
斜坡+在路上走：步高0.04 周期0.3 下沉步高0 步长0.03 偏航修正Kp0.015 最大修正步长0.06 位控 不加垂直于地面和任何俯仰修正 记得一换步态
微调步态：步高0.02 周期0.5 下沉步高0 步长0.007 偏航修正Kp0.015 最大修正步长0.008 力控 加垂直于地面修正

左二二状态拨杆：
拨上去keep 拨下来Walk

右二二状态拨杆：
拨上去正走 拨下来反走
*/

float std_step_length[3] = {0.1, 0.08, 0.035};
void GamePad_Control_Obs(void)
{
	if ((fabs(g_sbus_channels[2] - 1000.0f) <= 40) && (fabs(g_sbus_channels[3] - 200.0f)) <= 40 && (g_sbus_channels[5] == 192))
	{
		if (GetNextDogState() != Jump)
		{
			Change_Ctrl_State(Pos_Ctrl);
			Change_Dog_Pitch_State(Follow_Ground);
			ChangeDogState(Jump);
		}
	}
	else if ((fabs(g_sbus_channels[2] - 1000.0f) <= 40) && ((fabs(g_sbus_channels[3] - 1800.0f)) <= 40) && (g_sbus_channels[5] == 192))
	{
		if (GetNextDogState() != Backflip)
		{
			Change_Ctrl_State(Force_Ctrl);
			Change_Dog_Pitch_State(Follow_Dog);
			ChangeDogState(Backflip);
		}
	}
	
	else if ((fabs(g_sbus_channels[2] - 1800.0f) <= 40) && ((fabs(g_sbus_channels[3] - 1000.0f)) <= 40) && (g_sbus_channels[5] == 192))
	{
		SetDogChangeAble(ENABLE);
		ChangeDogState(Stand);
		SetDogChangeAble(ENABLE);
		ChangeDogState(Keep);
		Action_Param.stand_height[0] = 0.2f;
		Action_Param.stand_height[1] = 0.2f;
		Action_Param.stand_height[2] = 0.2f;
		Action_Param.stand_height[3] = 0.2f;
	}

	else if ((fabs(g_sbus_channels[2] - 200.0f) <= 40) && ((fabs(g_sbus_channels[3] - 1000.0f)) <= 40) && (g_sbus_channels[5] == 192))
	{
		Action_Param.stand_height[0] = 0.1475f;
		Action_Param.stand_height[1] = 0.1475f;
		Action_Param.stand_height[2] = 0.1475f;
		Action_Param.stand_height[3] = 0.1475f;
		SetDogChangeAble(ENABLE);
		ChangeDogState(Stand);
		SetDogChangeAble(ENABLE);
		ChangeDogState(Keep);
		Change_Ctrl_State(Pos_Ctrl);

	}
	else if ((fabs(g_sbus_channels[2] - 1000.0f) <= 40) && ((fabs(g_sbus_channels[3] - 1000.0f)) <= 40) && (g_sbus_channels[5] == 192))
	{
		if (GetNextDogState() != Keep)
		{
			Change_Dog_Pitch_State(Follow_Ground);			
			ChangeDogState(Keep);
		}
	}
	else if ((fabs(g_sbus_channels[2] - 1000.0f) <= 40) && ((fabs(g_sbus_channels[3] - 1000.0f)) <= 40) && (g_sbus_channels[5] == 1792))
	{
		if (GetNextDogState() != Walk)
		{
			Change_Dog_Pitch_State(Follow_Ground);			
			ChangeDogState(Walk);
		}
	}

	if ((g_sbus_channels[4] == 192) && (GetNowDogState() == Keep))
	{
		gait_param.step_height = 0.045f;
		gait_param.term = 0.8f;
		gait_param.step_height_down = 0;
		gait_param.std_step_length = 0.08f;
		gait_param.VMC_Euler_Walk.Kp_x = 0.0045f;
		gait_param.Yaw_max = 0.04;
		Change_Ctrl_State(Force_Ctrl);
		Change_Dog_Pitch_State(Follow_Ground);
	}
	else if ((g_sbus_channels[4] == 992) && (GetNowDogState() == Keep) && (Backfliptype != Second_Flip)) // 斜坡+在路上走
	{
		gait_param.step_height = 0.05f;
		gait_param.term = 0.3f;
		gait_param.step_height_down = 0;
		gait_param.std_step_length = 0.08f;
		gait_param.VMC_Euler_Walk.Kp_x = 0.0025f;
		gait_param.Yaw_max = 0.03f;
		Change_Ctrl_State(Pos_Ctrl);
		Change_Dog_Pitch_State(Follow_Dog);
	}
	else if ((g_sbus_channels[4] == 1792) && (GetNowDogState() == Keep)) // 微调步态
	{
		gait_param.step_height = 0.02f;
		gait_param.term = 0.4f;
		gait_param.step_height_down = 0;
		gait_param.std_step_length = 0.01f;
		gait_param.VMC_Euler_Walk.Kp_x = 0.0005;
		gait_param.Yaw_max = 0.008;
		Change_Ctrl_State(Pos_Ctrl);
		Change_Dog_Pitch_State(Follow_Ground);
	}

	if ((g_sbus_channels[6] == 1792) && (GetNowDogState() != Walk))
	{
		Set_Walk_Dire(Back);
	}
	else if ((g_sbus_channels[6] == 192) && (GetNowDogState() != Walk))
	{
		Set_Walk_Dire(Front);
	}
	
	float CHx = g_sbus_channels[0] - 1000;
	float CHy = g_sbus_channels[1] - 1000;

	if (sqrtf(CHx * CHx + CHy * CHy) < 200.0f)
	{
		Euler_Angle.Gamepad_Last_Yaw = 0;
		Euler_Angle.Target_Yaw = Euler_Angle.Yaw;
		gait_param.std_step_length = 0;
	}
	else
	{
		if (g_sbus_channels[4] == 192)
		{
			gait_param.std_step_length = std_step_length[0];
		}
		else if (g_sbus_channels[4] == 992)
		{
			gait_param.std_step_length = std_step_length[1];
		}
		else if (g_sbus_channels[4] == 1792)
		{
			gait_param.std_step_length = std_step_length[2];
		}
		float Dleta_Angle1, Dleta_Angle2;
		float now_ctrl_yaw = atan2f(-CHx, CHy) * 57.296f;
		if (fabs(now_ctrl_yaw) <= 15)
			now_ctrl_yaw = 0.00000001;
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


void GamePad_Control_Country(void)
{
	if ((fabs(g_sbus_channels[2] - 1000.0f) <= 40) && (fabs(g_sbus_channels[3] - 200.0f)) <= 40 && (g_sbus_channels[5]) == 192)
	{
		if (GetNextDogState() != Jump)
		{
			Change_Ctrl_State(Pos_Ctrl);
			Change_Dog_Pitch_State(Follow_Ground);
			ChangeDogState(Jump);
		}
	}
	else if ((fabs(g_sbus_channels[2] - 1000.0f) <= 40) && ((fabs(g_sbus_channels[3] - 1800.0f)) <= 40) && (g_sbus_channels[5]) == 192)
	{
//		if (GetNextDogState() != Backflip)
//		{
//			Change_Ctrl_State(Force_Ctrl);
//			Change_Dog_Pitch_State(Follow_Dog);
//			ChangeDogState(Backflip);
//		}
	}
	else if ((fabs(g_sbus_channels[2] - 1000.0f) <= 40) && ((fabs(g_sbus_channels[3] - 1000.0f)) <= 40) && (g_sbus_channels[5]) == 192)
	{
		if (GetNextDogState() != Keep)
		{
			Change_Dog_Pitch_State(Follow_Ground);			
			ChangeDogState(Keep);
		}
	}
	else if ((fabs(g_sbus_channels[2] - 1000.0f) <= 40) && ((fabs(g_sbus_channels[3] - 1000.0f)) <= 40) && (g_sbus_channels[5]) == 1792)
	{
		if (GetNextDogState() != Walk)
		{
			Change_Dog_Pitch_State(Follow_Ground);			
			ChangeDogState(Walk);
		}
	}

	if ((g_sbus_channels[4] == 192) && (GetNowDogState() == Keep))
	{
//		if(g_sbus_channels[7] == 192) // 下台阶
//		{
				gait_param.step_height = 0.05f;
				gait_param.term = 0.65f;
				gait_param.step_height_down = 0.005f;
				gait_param.std_step_length = 0.15f;
				gait_param.VMC_Euler_Walk.Kp_x = 0.0045f;
				gait_param.Yaw_max = 0.03f;
				Change_Ctrl_State(Force_Ctrl);
				Change_Dog_Pitch_State(Follow_Ground);
//		}
//		if(g_sbus_channels[7] == 992) // 双木桥走
//		{
//				gait_param.step_height = 0.02f;
//				gait_param.term = 0.5f;
//				gait_param.step_height_down = 0;
//				gait_param.std_step_length = 0.08f;
//				gait_param.VMC_Euler_Walk.Kp_x = 0.01;
//				gait_param.Yaw_max = 0.04;
//				Change_Ctrl_State(Pos_Ctrl);
//				Change_Dog_Pitch_State(Follow_Ground);
//		}
	}
	else if ((g_sbus_channels[4] == 992) && (GetNowDogState() == Keep) && (Backfliptype != Second_Flip)) // 斜坡+在路上走
	{
		gait_param.step_height = 0.05f;
		gait_param.term = 0.15f;
		gait_param.step_height_down = 0;
		gait_param.std_step_length = 0.15f;
		gait_param.VMC_Euler_Walk.Kp_x = 0.0025f;
		gait_param.Yaw_max = 0.03f;
		Change_Ctrl_State(Pos_Ctrl);
		Change_Dog_Pitch_State(Follow_Ground);
	}
	else if ((g_sbus_channels[4] == 1792) && (GetNowDogState() == Keep)) // 微调步态
	{
		gait_param.step_height = 0.04f;
		gait_param.term = 0.5f;
		gait_param.step_height_down = 0;
		gait_param.std_step_length = 0.007f;
		gait_param.VMC_Euler_Walk.Kp_x = 0.015;
		gait_param.Yaw_max = 0.008;
		Change_Ctrl_State(Pos_Ctrl);
		Change_Dog_Pitch_State(Follow_Ground);
	}

	if ((g_sbus_channels[6] == 1792) && (GetNowDogState() != Walk))
	{
		Set_Walk_Dire(Back);
	}
	else if ((g_sbus_channels[6] == 192) && (GetNowDogState() != Walk))
	{
		Set_Walk_Dire(Front);
	}
	
	float CHx = g_sbus_channels[0] - 1000;
	float CHy = g_sbus_channels[1] - 1000;

	if (sqrtf(CHx * CHx + CHy * CHy) < 200.0f)
	{
		Euler_Angle.Gamepad_Last_Yaw = 0;
		Euler_Angle.Target_Yaw = Euler_Angle.Yaw;
		gait_param.std_step_length = 0;
	}
	else
	{
		if (g_sbus_channels[4] == 192)
		{
			gait_param.std_step_length = std_step_length[0];
		}
		else if (g_sbus_channels[4] == 992)
		{
			gait_param.std_step_length = std_step_length[1];
		}
		else if (g_sbus_channels[4] == 1792)
		{
			gait_param.std_step_length = std_step_length[2];
		}
		float Dleta_Angle1, Dleta_Angle2;
		float now_ctrl_yaw = atan2f(-CHx, CHy) * 57.296f;
		if (fabs(now_ctrl_yaw) <= 15)
			now_ctrl_yaw = 0.00000001;
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

