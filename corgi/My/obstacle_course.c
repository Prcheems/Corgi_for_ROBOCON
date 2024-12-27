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
��ҡ�� ǰ������
��ҡ��:
					��
		��Ծ      ��շ�
					��
��շ���
��˫ľ���� ���ȶ���˫ľ���м��˵����м�
��������ԭ����������һ�εڶ��ζ���
̨�� �����ұ� ����ؽڴ���¥�ݲ���غ�

��Ծ��
GroundForward��¥���������� ����¥���м� ���忿���ֹ����ȥ
 ��̨�ף�����0.02 ����0.8 �³�����0 ����0.08 ƫ������Kp0.015 �����������0.06 ���� �Ӵ�ֱ�ڵ�������
б��+��·���ߣ�����0.04 ����0.3 �³�����0 ����0.03 ƫ������Kp0.015 �����������0.06 λ�� ���Ӵ�ֱ�ڵ�����κθ������� �ǵ�һ����̬
΢����̬������0.02 ����0.5 �³�����0 ����0.007 ƫ������Kp0.015 �����������0.008 ���� �Ӵ�ֱ�ڵ�������

�����״̬���ˣ�
����ȥkeep ������Walk

�Ҷ���״̬���ˣ�
����ȥ���� ����������
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
	else if ((g_sbus_channels[4] == 992) && (GetNowDogState() == Keep) && (Backfliptype != Second_Flip)) // б��+��·����
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
	else if ((g_sbus_channels[4] == 1792) && (GetNowDogState() == Keep)) // ΢����̬
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
//		if(g_sbus_channels[7] == 192) // ��̨��
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
//		if(g_sbus_channels[7] == 992) // ˫ľ����
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
	else if ((g_sbus_channels[4] == 992) && (GetNowDogState() == Keep) && (Backfliptype != Second_Flip)) // б��+��·����
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
	else if ((g_sbus_channels[4] == 1792) && (GetNowDogState() == Keep)) // ΢����̬
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

