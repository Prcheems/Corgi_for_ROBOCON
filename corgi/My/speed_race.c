
/*
ת��󲻷�����ʱ�䣨�Ӿ��ģ���
5 5 5
*/

// ��ɲ�ȶ���,�����˵�����ת���Ӿ��ж�����

#include "speed_race.h"
#include "dog.h"
#include "walk.h"
#include "imu.h"
#include "stm32h7xx.h"
#include "cmsis_os.h"
#include "walk.h"
#include "fsm.h"
#include "vision.h"
#include "pos_ctrl.h"
#include "delay_DWT.h"

extern int start_race_flag;
extern uint8_t change_step_length_flag; // �Ƿ�������������

#define offest 0
// ��д����ʱ�����߼�
speed_race_param Auto_spd = {
    // ����ش�����������ֵ
    .Front_dis = 2000,
    .Sides_dis = 2000,
	
    /*  
			��������
      һ 540  455  377
      �� 565  470  378
      �� 30  -60 		x
      �� 18  -40    x
    */
	
	  /* 
			��������
      һ x 			 x  				 x
      �� x  		 x   				 x
      �� x  100390(-100)   100376
      �� x  100393(-143)   100371
    */
    // �������
    .Max_left =    {540, 0, 565, 0,   30, 0,   18},
    .Target_side = {455, 0, 470, 0,  -90, 0,  -70, 100390},
    .Max_right =   {377, 0, 378, 0, -155, 0, -160, 100360},

    .Turn_Tirg = {40 + offest, 85 + offest, 70 + offest},

    // �жϹ��ߵ��ĸ�λ��
    .ID = 0,
    .stage_dire = {Front, Back, Back, Front, Front, Back, Back},

    // �Զ���·��־λ,1Ϊ��ʼ
    .start_race_flag = 0,

    .stage_yaw =		 {0, -47, -45, -90.01, -92, -35, -42},
    .std_stage_yaw = {0, -47, -45, -90.01, -92, -35, -42},

};

PID_vision Sides_PID = {
    .Kp = 15.0f,
    .Kd = 0.05f,
    .k = 0.5f,
};

/**
 * @brief ;�иı�ƫ���ǽ��к�������
 * @param PD
 * @param Auto_spd
 */
void Transverse_Correction(PID_vision *PD, speed_race_param *Auto_spd)
{
	// �������error
	if((fabs(Auto_spd->Sides_dis) >= 90000) && ((Auto_spd->ID == 4) || (Auto_spd->ID == 6)))
	{
		PD->err = (Auto_spd->Target_side[7] - Auto_spd->Sides_dis) / (100400 - Auto_spd->Max_right[7]);
	}
	else
	{
		PD->err = (Auto_spd->Target_side[Auto_spd->ID] - Auto_spd->Sides_dis) / (Auto_spd->Max_left[Auto_spd->ID] - Auto_spd->Max_right[Auto_spd->ID]);
	}
  // �ų�����Ӿ�����
  if (fabs(PD->err) >= 1.5f)
    PD->err = 0;

  // ���㲻��ȫD��
  PD->differ = PD->Kd * (1 - PD->k) * (PD->err - PD->last_err) + PD->k * PD->last_err;

  // �������
  float output = PD->Kp * PD->err + PD->differ;

  PD->last_err = PD->err;
  PD->differ = PD->last_differ;

  Auto_spd->stage_yaw[Auto_spd->ID] = output + Auto_spd->std_stage_yaw[Auto_spd->ID];
}

uint32_t time_now = 0; // ��λ����
uint32_t time_increment = 2;

float step_length_start = 0.02f;
float step_length_very_fast = 0.178f; // ֱ��ʱ���Ӳ�������
float step_length_fast = 0.165f;      // ֱ��ʱ���Ӳ�������
float step_length_slow = 0.105f;      // Ҫת��ʱ���ٲ��������������
float step_length_very_slow = 0.005f;

float time_start = 400;
float time1 = 2100; // ��һ��ֱ�߶����ÿ��ٲ�����ʱ��
float time2_start = 400;
float time2 = 2800; // �ڶ���......
float time3_start = 400;
float time3 = 5000;
float time4_start = 400;
float time4 = 3500;

float time_stop_vision_1 = 1500;
float time_stop_vision_2 = 2500;
uint8_t stop_vision_flag = 0;
uint8_t clear_time_now = 1; // ��־λ����������time_now

float dead_band = 25;

void Stage_Judge(speed_race_param *Auto_spd)
{
  // ����ID:
  // ID = 0,������,����ʼλ���ߵ��ش���1
  // ID = 1,������,�ӱش���1ת45��
  // ID = 2,������,�ӱش���1�ߵ��ش���2
  // ID = 3,������,�ӱش���2ת45��
  // ID = 4,������,�ӱش���2�ߵ��ش���3
  // ID = 5,������,�ӱش���3ת45��
  // ID = 6,������,�ӱش���3�ߵ��ش���4
  // ID = 7,stop
  if (Auto_spd->ID == 0)
  {
		// ID = 0,������,����ʼλ���ߵ��ش���1
    Set_Walk_Length(step_length_very_fast);   
    Set_Turning_Kp(0.002f);
    Sides_PID.Kp = 15;
    Sides_PID.Kd = 0.1;
		if (time_now <= time_start)
      Set_Walk_Length(step_length_start);
		else if ((time_now >= time_start) && (time_now <= time1))
			Set_Walk_Length(step_length_very_fast);
    if (time_now >= time1)
      Set_Walk_Length(step_length_slow);
  }
	
	
  if ((Auto_spd->ID == 0) && ((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[0]) || (Auto_spd->Camera_dire_flag == -1)) && (Auto_spd->Front_dis > 0) && (time_now >= 1000))
  {
		ChangeDogState(Keep);
		osDelay(300);
		
    SetDogChangeAble(ENABLE);		
		ChangeDogState(Walk);
    // ID = 1,������,�ӱش���1ת45��
    Set_Walk_Length(step_length_very_slow);
    Auto_spd->ID = 1;
    gait_param.VMC_Euler_Walk.last_differ_x = 0;
    gait_param.VMC_Euler_Walk.last_err_x = 0;
    Set_Turning_Kp(0.004f);
  }

	
  if ((Auto_spd->ID == 1) && (Euler_Angle.Yaw <= Auto_spd->std_stage_yaw[Auto_spd->ID]))
  {
    // ID = 2,������,�ӱش���1�ߵ��ش���2
    if (clear_time_now == 1)
    {
      time_now = 0;
      clear_time_now = 2;
    }
    Sides_PID.Kp = 15;
    Sides_PID.Kd = 0.1;

    Auto_spd->ID = 2;
    Set_Turning_Kp(0.0025f); // 0.0007
  }
	if ((Auto_spd->ID == 2) && (time_now <= time2_start))
		Set_Walk_Length(step_length_slow);
  else if ((Auto_spd->ID == 2) && (time_now >= time2_start) && (time_now <= time2))
    Set_Walk_Length(step_length_very_fast);
  else if ((Auto_spd->ID == 2) && (time_now >= time2))
    Set_Walk_Length(step_length_slow);
	
	
  if ((Auto_spd->ID == 2) && ((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[1]) || (Auto_spd->Camera_dire_flag == 1)) && (time_now >= 1000))
  {	
		ChangeDogState(Keep);
		osDelay(300);
		
    SetDogChangeAble(ENABLE);		
		ChangeDogState(Walk);
    // ID = 3,������,�ӱش���2ת45��
    Set_Walk_Length(step_length_very_slow);
    Sides_PID.Kp = 15;
    Sides_PID.Kd = 0.1;
    Auto_spd->ID = 3;
    Set_Turning_Kp(0.0025f);
  }

	
  if ((Auto_spd->ID == 3) && (Euler_Angle.Yaw <= Auto_spd->std_stage_yaw[Auto_spd->ID]))
  {
    // ID = 4,������,�ӱش���2�ߵ��ش���3
    if (clear_time_now == 2)
    {
      time_now = 0;
      clear_time_now = 3;
    }
    Sides_PID.Kp = 10;  // 12
    Sides_PID.Kd = 0.1; // 0.5
    Auto_spd->ID = 4;
    Set_Turning_Kp(0.002f); // 0.0008
  }
	if ((Auto_spd->ID == 4) && (time_now <= time3_start))
    Set_Walk_Length(step_length_very_fast);
  else if ((Auto_spd->ID == 4) && (time_now >= time3_start) && (time_now <= time3))
    Set_Walk_Length(step_length_very_fast);
  else if ((Auto_spd->ID == 4) && (time_now >= time3))
    Set_Walk_Length(step_length_slow);
	
	
//////  if ((Auto_spd->ID == 4) && (((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[2]) && (Auto_spd->Front_dis > 0)) || (Auto_spd->Camera_dire_flag == -1)))
//////  {	
//////		Auto_spd->ID = 5;
//////    if (clear_time_now == 3)
//////    {
//////      time_now = 0;
//////      clear_time_now = 4;
//////    }		
//////  }
//////	if ((time_now >= 2000) && (Auto_spd->ID == 5))
//////	{
//////		ChangeDogState(Keep);
//////		osDelay(300);
//////    SetDogChangeAble(ENABLE);		
//////		ChangeDogState(Walk);
//////    // ID = 5,������,�ӱش���3ת45��
//////    Set_Walk_Length(step_length_very_slow);
//////    Sides_PID.Kp = 15;  // 12
//////    Sides_PID.Kd = 0.5; // 0.5
//////    Set_Turning_Kp(0.0035f);	
//////	}
  if ((Auto_spd->ID == 4) && (((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[2]) && (Auto_spd->Front_dis > 0)) || (Auto_spd->Camera_dire_flag == -1)) && (time_now >= 1000))
  {	
		ChangeDogState(Keep);
		osDelay(300);
    SetDogChangeAble(ENABLE);		
		ChangeDogState(Walk);
    // ID = 5,������,�ӱش���3ת45��
    Set_Walk_Length(step_length_very_slow);
    Sides_PID.Kp = 15;  // 12
    Sides_PID.Kd = 0.5; // 0.5
    Set_Turning_Kp(0.0035f);	
		Auto_spd->ID = 5;
    if (clear_time_now == 3)
    {
      time_now = 0;
      clear_time_now = 4;
    }		
  }

	
  if ((Auto_spd->ID == 5) && (Euler_Angle.Yaw >= Auto_spd->std_stage_yaw[Auto_spd->ID]))
  {
    // ID = 6,������,�ӱش���3�ߵ��ش���4
    if (clear_time_now == 4)
    {
      time_now = 0;
      clear_time_now = 5;
    }
    Auto_spd->ID = 6;
    Sides_PID.Kp = 15;
    Sides_PID.Kd = 0.1;
    Set_Turning_Kp(0.0025f);
  }	
	if ((Auto_spd->ID == 6) && (time_now <= time4_start))
    Set_Walk_Length(step_length_very_fast);
  else if ((Auto_spd->ID == 6) && (time_now >= time4_start) && (time_now <= time4))
    Set_Walk_Length(step_length_very_fast);
  else if ((Auto_spd->ID == 6) && (time_now >= time4))
    Set_Walk_Length(step_length_very_fast);

	
  if ((Auto_spd->ID == 6) && (Auto_spd->Front_dis <= 100) && (Auto_spd->Front_dis > 0) && (time_now >= 1000))
  {
    Auto_spd->ID = 7;
  }
}
int Sides_cnt = 0;
float race_timer = 0;
void start_race(void)
{
	Action_Param.keep_type = 1;
  Stage_Judge(&Auto_spd);
  Set_Walk_Dire(Auto_spd.stage_dire[Auto_spd.ID]);
  if ((leg[0].ctrl_state == Force_Ctrl) && (Auto_spd.ID == 0) && (GetNowDogState() != Stand) && (GetNowDogState() != Squat))
  {
    Change_Ctrl_State(Pos_Ctrl);
  }
  if ((Auto_spd.start_race_flag == 1) && (GetNowDogState() != Walk))
  {  
    Auto_spd.start_race_flag = 2;
    ChangeDogState(Walk);
    SetDogChangeAble(ENABLE);
    Auto_spd.ID = 0;
    clear_time_now = 1;
    Yaw_Reset(&Euler_Angle);
  }
  if ((Auto_spd.ID == 7) && (GetNowDogState() == Walk))
  {
    SetDogChangeAble(ENABLE);
    ChangeDogState(Keep);
		race_timer = Time_Points_Param.Now_Tick;
  }

  if ((((Auto_spd.ID == 0) && (time_now >= 600)) || Auto_spd.ID == 2 || Auto_spd.ID == 4 || Auto_spd.ID == 6) && (fabs(Auto_spd.Sides_dis - Auto_spd.Target_side[Auto_spd.ID]) > dead_band))
  {
			if ((((Auto_spd.Turn_Tirg[Auto_spd.ID] - Auto_spd.Front_dis) >= 300) && (Auto_spd.Front_dis != 2000)) || (Auto_spd.Front_dis == 2000))
			{
					if (Auto_spd.Sides_dis != 2000)
					{
						Transverse_Correction(&Sides_PID, &Auto_spd);
					}
					else if ((Auto_spd.Sides_dis == 2000) && (Sides_cnt) >= 120)
					{
						Auto_spd.stage_yaw[Auto_spd.ID] = Auto_spd.std_stage_yaw[Auto_spd.ID];
						Sides_cnt = 0;
					}
					else
					{
						Sides_cnt++;
					}
			}
	}
	if (Auto_spd.ID != 7)
	{
		Set_Target_Yaw(Auto_spd.stage_yaw[Auto_spd.ID]);
	}
  time_now += time_increment;
}



///////*
//////7.7 nignt
//////5 5 5
//////*/

//////// ��ɲ�ȶ���,�����˵�����ת���Ӿ��ж�����

//////#include "speed_race.h"
//////#include "dog.h"
//////#include "walk.h"
//////#include "imu.h"
//////#include "stm32h7xx.h"
//////#include "cmsis_os.h"
//////#include "walk.h"
//////#include "fsm.h"
//////#include "vision.h"
//////#include "pos_ctrl.h"
//////#include "delay_DWT.h"

//////extern int start_race_flag;
//////extern uint8_t change_step_length_flag; // �Ƿ�������������

//////#define offest 0
//////// ��д����ʱ�����߼�
//////speed_race_param Auto_spd = {
//////    // ����ش�����������ֵ
//////    .Front_dis = 2000,
//////    .Sides_dis = 2000,
//////	
//////    /*  
//////			��������
//////      һ 540  455  377
//////      �� 565  470  378
//////      �� 30  -60 		x
//////      �� 18  -40    x
//////    */
//////	
//////	  /* 
//////			��������
//////      һ x 			 x  				 x
//////      �� x  		 x   				 x
//////      �� x  100390(-100)   100376
//////      �� x  100393(-143)   100371
//////    */
//////    // �������
//////    .Max_left =    {540, 0, 565, 0,   30, 0,   18},
//////    .Target_side = {455, 0, 470, 0,  -60, 0,  -60, 100390},
//////    .Max_right =   {377, 0, 378, 0, -155, 0, -160, 100375},

//////    .Turn_Tirg = {40 + offest, 80 + offest, 40 + offest},

//////    // �жϹ��ߵ��ĸ�λ��
//////    .ID = 0,
//////    .stage_dire = {Front, Back, Back, Front, Front, Back, Back},

//////    // �Զ���·��־λ,1Ϊ��ʼ
//////    .start_race_flag = 0,

//////    .stage_yaw =		 {0, -47, -45, -95, -90.02, -40, -43},
//////    .std_stage_yaw = {0, -47, -45, -95, -90.02, -40, -43},

//////};

//////PID_vision Sides_PID = {
//////    .Kp = 15.0f,
//////    .Kd = 0.05f,
//////    .k = 0.5f,
//////};

///////**
////// * @brief ;�иı�ƫ���ǽ��к�������
////// * @param PD
////// * @param Auto_spd
////// */
//////void Transverse_Correction(PID_vision *PD, speed_race_param *Auto_spd)
//////{
//////	// �������error
//////	if((fabs(Auto_spd->Sides_dis) >= 90000) && (/*(Auto_spd->ID == 4) || */(Auto_spd->ID == 6)))
//////	{
//////		PD->err = (Auto_spd->Target_side[7] - Auto_spd->Sides_dis) / (100400 - Auto_spd->Max_right[7]);
//////	}
//////	else
//////	{
//////		PD->err = (Auto_spd->Target_side[Auto_spd->ID] - Auto_spd->Sides_dis) / (Auto_spd->Max_left[Auto_spd->ID] - Auto_spd->Max_right[Auto_spd->ID]);
//////	}
//////  // �ų�����Ӿ�����
//////  if (fabs(PD->err) >= 1.0f)
//////    PD->err = 0;

//////  // ���㲻��ȫD��
//////  PD->differ = PD->Kd * (1 - PD->k) * (PD->err - PD->last_err) + PD->k * PD->last_err;

//////  // �������
//////  float output = PD->Kp * PD->err + PD->differ;

//////  PD->last_err = PD->err;
//////  PD->differ = PD->last_differ;

//////  Auto_spd->stage_yaw[Auto_spd->ID] = output + Auto_spd->std_stage_yaw[Auto_spd->ID];
//////}

//////uint32_t time_now = 0; // ��λ����
//////uint32_t time_increment = 2;

//////float step_length_start = 0.02f;
//////float step_length_very_fast = 0.178f; // ֱ��ʱ���Ӳ�������
//////float step_length_fast = 0.165f;      // ֱ��ʱ���Ӳ�������
//////float step_length_slow = 0.105f;      // Ҫת��ʱ���ٲ��������������
//////float step_length_very_slow = 0.005f;

//////float time_start = 400;
//////float time1 = 1800; // ��һ��ֱ�߶����ÿ��ٲ�����ʱ��
//////float time2_start = 400;
//////float time2 = 2800; // �ڶ���......
//////float time3_start = 400;
//////float time3 = 5000;
//////float time4_start = 400;
//////float time4 = 3500;

//////float time_stop_vision_1 = 1500;
//////float time_stop_vision_2 = 2500;
//////uint8_t stop_vision_flag = 0;
//////uint8_t clear_time_now = 1; // ��־λ����������time_now

//////float dead_band = 25;

//////void Stage_Judge(speed_race_param *Auto_spd)
//////{
//////  // ����ID:
//////  // ID = 0,������,����ʼλ���ߵ��ش���1
//////  // ID = 1,������,�ӱش���1ת45��
//////  // ID = 2,������,�ӱش���1�ߵ��ش���2
//////  // ID = 3,������,�ӱش���2ת45��
//////  // ID = 4,������,�ӱش���2�ߵ��ش���3
//////  // ID = 5,������,�ӱش���3ת45��
//////  // ID = 6,������,�ӱش���3�ߵ��ش���4
//////  // ID = 7,stop
//////  if (Auto_spd->ID == 0)
//////  {
//////		// ID = 0,������,����ʼλ���ߵ��ش���1
//////    Set_Walk_Length(step_length_very_fast);   
//////    Set_Turning_Kp(0.002f);
//////    Sides_PID.Kp = 15;
//////    Sides_PID.Kd = 0.1;
//////		if (time_now <= time_start)
//////      Set_Walk_Length(step_length_start);
//////		else if ((time_now >= time_start) && (time_now <= time1))
//////			Set_Walk_Length(step_length_very_fast);
//////    if (time_now >= time1)
//////      Set_Walk_Length(step_length_slow);
//////  }
//////	
//////	
//////  if ((Auto_spd->ID == 0) && ((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[0]) || (Auto_spd->Camera_dire_flag == -1)) && (Auto_spd->Front_dis > 0))
//////  {
//////		ChangeDogState(Keep);
//////		osDelay(300);
//////		
//////    SetDogChangeAble(ENABLE);		
//////		ChangeDogState(Walk);
//////    // ID = 1,������,�ӱش���1ת45��
//////    Set_Walk_Length(step_length_very_slow);
//////    Auto_spd->ID = 1;
//////    gait_param.VMC_Euler_Walk.last_differ_x = 0;
//////    gait_param.VMC_Euler_Walk.last_err_x = 0;
//////    Set_Turning_Kp(0.004f);
//////  }

//////	
//////  if ((Auto_spd->ID == 1) && (Euler_Angle.Yaw <= Auto_spd->std_stage_yaw[Auto_spd->ID]))
//////  {
//////    // ID = 2,������,�ӱش���1�ߵ��ش���2
//////    if (clear_time_now == 1)
//////    {
//////      time_now = 0;
//////      clear_time_now = 2;
//////    }
//////    Sides_PID.Kp = 15;
//////    Sides_PID.Kd = 0.1;

//////    Auto_spd->ID = 2;
//////    Set_Turning_Kp(0.0025f); // 0.0007
//////  }
//////	if ((Auto_spd->ID == 2) && (time_now <= time2_start))
//////		Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 2) && (time_now >= time2_start) && (time_now <= time2))
//////    Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 2) && (time_now >= time2))
//////    Set_Walk_Length(step_length_slow);
//////	
//////	
//////  if ((Auto_spd->ID == 2) && ((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[1]) || (Auto_spd->Camera_dire_flag == 1)))
//////  {	
//////		ChangeDogState(Keep);
//////		osDelay(300);
//////		
//////    SetDogChangeAble(ENABLE);		
//////		ChangeDogState(Walk);
//////    // ID = 3,������,�ӱش���2ת45��
//////    Set_Walk_Length(step_length_very_slow);
//////    Sides_PID.Kp = 15;
//////    Sides_PID.Kd = 0.1;
//////    Auto_spd->ID = 3;
//////    Set_Turning_Kp(0.0035f);
//////  }

//////	
//////  if ((Auto_spd->ID == 3) && (Euler_Angle.Yaw <= Auto_spd->std_stage_yaw[Auto_spd->ID]))
//////  {
//////    // ID = 4,������,�ӱش���2�ߵ��ش���3
//////    if (clear_time_now == 2)
//////    {
//////      time_now = 0;
//////      clear_time_now = 3;
//////    }
//////    Sides_PID.Kp = 15;  // 12
//////    Sides_PID.Kd = 0.1; // 0.5
//////    Auto_spd->ID = 4;
//////    Set_Turning_Kp(0.0035f); // 0.0008
//////  }
//////	if ((Auto_spd->ID == 4) && (time_now <= time3_start))
//////    Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 4) && (time_now >= time3_start) && (time_now <= time3))
//////    Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 4) && (time_now >= time3))
//////    Set_Walk_Length(((step_length_slow + step_length_very_slow) / 2.0f));
//////	if((Auto_spd->ID == 4) && (time_now >= 2500))
//////		Auto_spd->Target_side[4] = -110;
//////	
//////  if ((Auto_spd->ID == 4) && (((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[2]) && (Auto_spd->Front_dis > 0)) || (Auto_spd->Camera_dire_flag == -1)))
//////  {	
//////		Auto_spd->ID = 5;
//////    if (clear_time_now == 3)
//////    {
//////      time_now = 0;
//////      clear_time_now = 4;
//////    }		
//////  }
//////	if ((time_now >= 2000) && (Auto_spd->ID == 5))
//////	{
//////		ChangeDogState(Keep);
//////		osDelay(300);
//////    SetDogChangeAble(ENABLE);		
//////		ChangeDogState(Walk);
//////    // ID = 5,������,�ӱش���3ת45��
//////    Set_Walk_Length(step_length_very_slow);
//////    Sides_PID.Kp = 15;  // 12
//////    Sides_PID.Kd = 0.5; // 0.5
//////    Set_Turning_Kp(0.0035f);	
//////	
//////	}
//////	
//////  if ((Auto_spd->ID == 5) && (Euler_Angle.Yaw >= Auto_spd->std_stage_yaw[Auto_spd->ID]))
//////  {
//////    // ID = 6,������,�ӱش���3�ߵ��ش���4
//////    if (clear_time_now == 4)
//////    {
//////      time_now = 0;
//////      clear_time_now = 5;
//////    }
//////    Auto_spd->ID = 6;
//////    Sides_PID.Kp = 15;
//////    Sides_PID.Kd = 0.1;
//////    Set_Turning_Kp(0.0025f);
//////  }	
//////	if ((Auto_spd->ID == 6) && (time_now <= time4_start))
//////    Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 6) && (time_now >= time4_start) && (time_now <= time4))
//////    Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 6) && (time_now >= time4))
//////    Set_Walk_Length(step_length_very_fast);

//////	
//////  if ((Auto_spd->ID == 6) && (Auto_spd->Front_dis <= 100) && (Auto_spd->Front_dis > 0))
//////  {
//////    Auto_spd->ID = 7;
//////  }
//////}
//////int Sides_cnt = 0;
//////float race_timer = 0;
//////void start_race(void)
//////{
//////	Action_Param.keep_type = 1;
//////  Stage_Judge(&Auto_spd);
//////  Set_Walk_Dire(Auto_spd.stage_dire[Auto_spd.ID]);
//////  if ((leg[0].ctrl_state == Force_Ctrl) && (Auto_spd.ID == 0) && (GetNowDogState() != Stand) && (GetNowDogState() != Squat))
//////  {
//////    Change_Ctrl_State(Pos_Ctrl);
//////  }
//////  if ((Auto_spd.start_race_flag == 1) && (GetNowDogState() != Walk))
//////  {  
//////    Auto_spd.start_race_flag = 2;
//////    ChangeDogState(Walk);
//////    SetDogChangeAble(ENABLE);
//////    Auto_spd.ID = 0;
//////    clear_time_now = 1;
//////    Yaw_Reset(&Euler_Angle);
//////  }
//////  if ((Auto_spd.ID == 7) && (GetNowDogState() == Walk))
//////  {
//////    SetDogChangeAble(ENABLE);
//////    ChangeDogState(Keep);
//////		race_timer = Time_Points_Param.Now_Tick;
//////  }

//////  if ((((Auto_spd.ID == 0) && (time_now >= 600)) || Auto_spd.ID == 2 || Auto_spd.ID == 4 || Auto_spd.ID == 6) && (fabs(Auto_spd.Sides_dis - Auto_spd.Target_side[Auto_spd.ID]) > dead_band))
//////  {
//////			if ((((Auto_spd.Turn_Tirg[Auto_spd.ID] - Auto_spd.Front_dis) >= 300) && (Auto_spd.Front_dis != 2000)) || (Auto_spd.Front_dis == 2000))
//////			{
//////					if (Auto_spd.Sides_dis != 2000)
//////					{
//////						Transverse_Correction(&Sides_PID, &Auto_spd);
//////					}
//////					else if ((Auto_spd.Sides_dis == 2000) && (Sides_cnt) >= 120)
//////					{
//////						Auto_spd.stage_yaw[Auto_spd.ID] = Auto_spd.std_stage_yaw[Auto_spd.ID];
//////						Sides_cnt = 0;
//////					}
//////					else
//////					{
//////						Sides_cnt++;
//////					}
//////			}
//////	}
//////	if (Auto_spd.ID != 7)
//////	{
//////		Set_Target_Yaw(Auto_spd.stage_yaw[Auto_spd.ID]);
//////	}
//////  time_now += time_increment;
//////}




/*
ת��󲻷�����ʱ�䣨�Ӿ��ģ���
6 5 7
*/

// ��ɲ�ȶ��棬ǰ����ת�䶼�����ȶ���ֻ�е�����ת���и����߲����ش��������߳�ȥ�����û�кõĽ���취
//////#include "speed_race.h"
//////#include "dog.h"
//////#include "walk.h"
//////#include "imu.h"
//////#include "stm32h7xx.h"
//////#include "cmsis_os.h"
//////#include "walk.h"
//////#include "fsm.h"
//////#include "vision.h"
//////#include "pos_ctrl.h"
//////#include "delay_DWT.h"

//////extern int start_race_flag;
//////extern uint8_t change_step_length_flag; // �Ƿ�������������

//////#define offest 0
//////// ��д����ʱ�����߼�
//////speed_race_param Auto_spd = {
//////    // ����ش�����������ֵ
//////    .Front_dis = 2000,
//////    .Sides_dis = 2000,
//////    /*
//////      һ 610  490  365
//////      �� 520  420  320
//////      �� 40  -100 -180
//////         40  -90  -170
//////      �� 45  -70  -140
//////    */
//////    // �������
//////    .Max_left =    {610 + offest, 0, 580 + offest, 0,   40 + offest, 0,   45 + offest},
//////    .Target_side = {490 + offest, 0, 470 + offest, 0, -125 + offest, 0,  -70 + offest},
//////    .Max_right =   {365 + offest, 0, 340 + offest, 0, -175 + offest, 0, -190 + offest},

//////    .Turn_Tirg = {50 + offest, 100 + offest, 475	+ offest},

//////    // �жϹ��ߵ��ĸ�λ��
//////    .ID = 0,
//////    .stage_dire = {Front, Back, Back, Front, Front, Back, Back},

//////    // �Զ���·��־λ,1Ϊ��ʼ
//////    .start_race_flag = 0,

//////    .stage_yaw =		 {-2, -52, -45, -100, -89, -35, -43},
//////    .std_stage_yaw = {-2, -52, -45, -100, -89, -35, -43},

//////};

//////PID_vision Sides_PID = {
//////    .Kp = 15.0f,
//////    .Kd = 0.05f,
//////    .k = 0.5f,
//////};

///////**
////// * @brief ;�иı�ƫ���ǽ��к�������
////// * @param PD
////// * @param Auto_spd
////// */
//////void Transverse_Correction(PID_vision *PD, speed_race_param *Auto_spd)
//////{
//////  // �������error
//////  PD->err = (Auto_spd->Target_side[Auto_spd->ID] - Auto_spd->Sides_dis) / (Auto_spd->Max_left[Auto_spd->ID] - Auto_spd->Max_right[Auto_spd->ID]);

//////  // �ų�����Ӿ�����
//////  if (fabs(PD->err) >= 1.0f)
//////    PD->err = 0;

//////  // ���㲻��ȫD��
//////  PD->differ = PD->Kd * (1 - PD->k) * (PD->err - PD->last_err) + PD->k * PD->last_err;

//////  // �������
//////  float output = PD->Kp * PD->err + PD->differ;

//////  PD->last_err = PD->err;
//////  PD->differ = PD->last_differ;

//////  Auto_spd->stage_yaw[Auto_spd->ID] = output + Auto_spd->std_stage_yaw[Auto_spd->ID];
//////}

//////uint32_t time_now = 0; // ��λ����
//////uint32_t time_increment = 2;

//////float step_length_start = 0.02f;
//////float step_length_very_fast = 0.178f; // ֱ��ʱ���Ӳ�������
//////float step_length_fast = 0.165f;      // ֱ��ʱ���Ӳ�������
//////float step_length_slow = 0.105f;      // Ҫת��ʱ���ٲ��������������
//////float step_length_very_slow = 0.025f;

//////float time_start = 400;
//////float time1 = 1800; // ��һ��ֱ�߶����ÿ��ٲ�����ʱ��
//////float time2_start = 400;
//////float time2 = 2500; // �ڶ���......
//////float time3_start = 400;
//////float time3 = 5000;
//////float time4_start = 400;
//////float time4 = 3500;

//////float time_stop_vision_1 = 1500;
//////float time_stop_vision_2 = 2500;
//////uint8_t stop_vision_flag = 0;
//////uint8_t clear_time_now = 1; // ��־λ����������time_now

//////float dead_band = 25;

//////void Stage_Judge(speed_race_param *Auto_spd)
//////{
//////  // ����ID:
//////  // ID = 0,������,����ʼλ���ߵ��ش���1
//////  // ID = 1,������,�ӱش���1ת45��
//////  // ID = 2,������,�ӱش���1�ߵ��ش���2
//////  // ID = 3,������,�ӱش���2ת45��
//////  // ID = 4,������,�ӱش���2�ߵ��ش���3
//////  // ID = 5,������,�ӱش���3ת45��
//////  // ID = 6,������,�ӱش���3�ߵ��ش���4
//////  // ID = 7,stop
//////  if (Auto_spd->ID == 0)
//////  {
//////		// ID = 0,������,����ʼλ���ߵ��ش���1
//////    Set_Walk_Length(step_length_very_fast);   
//////    Set_Turning_Kp(0.002f);
//////    Sides_PID.Kp = 15;
//////    Sides_PID.Kd = 0.1;
//////		if (time_now <= time_start)
//////      Set_Walk_Length(step_length_start);
//////		else if ((time_now >= time_start) && (time_now <= time1))
//////			Set_Walk_Length(step_length_very_fast);
//////    if (time_now >= time1)
//////      Set_Walk_Length(step_length_slow);
//////  }
//////	
//////	
//////  if ((Auto_spd->ID == 0) && ((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[0]) || (Auto_spd->Camera_dire_flag == -1)) && (Auto_spd->Front_dis > 0))
//////  {
//////		ChangeDogState(Start);
//////		osDelay(300);
//////		
//////    SetDogChangeAble(ENABLE);		
//////		ChangeDogState(Walk);
//////    // ID = 1,������,�ӱش���1ת45��
//////    Set_Walk_Length(step_length_very_slow);
//////    Auto_spd->ID = 1;
//////    gait_param.VMC_Euler_Walk.last_differ_x = 0;
//////    gait_param.VMC_Euler_Walk.last_err_x = 0;
//////    Set_Turning_Kp(0.004f);
//////  }

//////	
//////  if ((Auto_spd->ID == 1) && (Euler_Angle.Yaw <= Auto_spd->std_stage_yaw[Auto_spd->ID]))
//////  {
//////    // ID = 2,������,�ӱش���1�ߵ��ش���2
//////    if (clear_time_now == 1)
//////    {
//////      time_now = 0;
//////      clear_time_now = 2;
//////    }
//////    Sides_PID.Kp = 15;
//////    Sides_PID.Kd = 0.1;

//////    Set_Walk_Length(step_length_slow);
//////    Auto_spd->ID = 2;
//////    Set_Turning_Kp(0.0025f); // 0.0007
//////  }
//////	if ((Auto_spd->ID == 2) && (time_now <= time2_start))
//////		Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 2) && (time_now >= time2_start) && (time_now <= time2))
//////    Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 2) && (time_now >= time2))
//////    Set_Walk_Length(step_length_slow);
//////	
//////	
//////  if ((Auto_spd->ID == 2) && ((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[1]) || (Auto_spd->Camera_dire_flag == 1)))
//////  {	
//////		ChangeDogState(Start);
//////		osDelay(300);
//////		
//////    SetDogChangeAble(ENABLE);		
//////		ChangeDogState(Walk);
//////    // ID = 3,������,�ӱش���2ת45��
//////    Set_Walk_Length(step_length_very_slow);
//////    Sides_PID.Kp = 15;
//////    Sides_PID.Kd = 0.1;
//////    Auto_spd->ID = 3;
//////    Set_Turning_Kp(0.0035f);
//////  }

//////	
//////  if ((Auto_spd->ID == 3) && (Euler_Angle.Yaw <= Auto_spd->std_stage_yaw[Auto_spd->ID]))
//////  {
//////    // ID = 4,������,�ӱش���2�ߵ��ش���3
//////    if (clear_time_now == 2)
//////    {
//////      time_now = 0;
//////      clear_time_now = 3;
//////    }
//////    Sides_PID.Kp = 15;  // 12
//////    Sides_PID.Kd = 0.1; // 0.5
//////    Auto_spd->ID = 4;
//////    Set_Walk_Length(step_length_very_fast);
//////    Set_Turning_Kp(0.0025f); // 0.0008
//////  }
//////	if ((Auto_spd->ID == 4) && (time_now <= time3_start))
//////    Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 4) && (time_now >= time3_start) && (time_now <= time3))
//////    Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 4) && (time_now >= time3))
//////    Set_Walk_Length(((step_length_slow + step_length_very_slow) / 2.0f));
//////	
//////	
//////  if ((Auto_spd->ID == 4) && (((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[2]) && (Auto_spd->Front_dis > 0)) || (Auto_spd->Camera_dire_flag == -1)))
//////  {
//////		ChangeDogState(Start);
//////		osDelay(300);
//////		
//////    SetDogChangeAble(ENABLE);		
//////		ChangeDogState(Walk);
//////    // ID = 5,������,�ӱش���3ת45��
//////    Set_Walk_Length(step_length_very_slow);
//////    Sides_PID.Kp = 15;  // 12
//////    Sides_PID.Kd = 0.5; // 0.5
//////    Auto_spd->ID = 5;
//////    Set_Turning_Kp(0.0035f);
//////  }

//////	
//////  if ((Auto_spd->ID == 5) && (Euler_Angle.Yaw >= Auto_spd->std_stage_yaw[Auto_spd->ID]))
//////  {
//////    // ID = 6,������,�ӱش���3�ߵ��ش���4
//////    if (clear_time_now == 3)
//////    {
//////      time_now = 0;
//////      clear_time_now = 4;
//////    }
//////    Auto_spd->ID = 6;
//////    Set_Walk_Length(step_length_very_fast);
//////    Sides_PID.Kp = 15;
//////    Sides_PID.Kd = 0.1;
//////    Set_Turning_Kp(0.0025f);
//////  }	
//////	if ((Auto_spd->ID == 6) && (time_now <= time4_start))
//////    Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 6) && (time_now >= time4_start) && (time_now <= time4))
//////    Set_Walk_Length(step_length_very_fast);
//////  else if ((Auto_spd->ID == 6) && (time_now >= time4))
//////    Set_Walk_Length(step_length_very_fast);

//////	
//////  if ((Auto_spd->ID == 6) && (Auto_spd->Front_dis <= 100) && (Auto_spd->Front_dis > 0))
//////  {
//////    Auto_spd->ID = 7;
//////  }
//////}
//////int Sides_cnt = 0;
//////float race_timer = 0;
//////void start_race(void)
//////{
//////  Stage_Judge(&Auto_spd);
//////  Set_Walk_Dire(Auto_spd.stage_dire[Auto_spd.ID]);
//////  if ((leg[0].ctrl_state == Force_Ctrl) && (Auto_spd.ID == 0) && (GetNowDogState() != Stand) && (GetNowDogState() != Squat))
//////  {
//////    Change_Ctrl_State(Pos_Ctrl);
//////  }
//////  if ((Auto_spd.start_race_flag == 1) && (GetNowDogState() != Walk))
//////  {  
//////    Auto_spd.start_race_flag = 2;
//////    ChangeDogState(Walk);
//////    SetDogChangeAble(ENABLE);
//////    Auto_spd.ID = 0;
//////    clear_time_now = 1;
//////    Yaw_Reset(&Euler_Angle);
//////  }
//////  if ((Auto_spd.ID == 7) && (GetNowDogState() == Walk))
//////  {
//////    SetDogChangeAble(ENABLE);
//////    ChangeDogState(Keep);
//////		race_timer = Time_Points_Param.Now_Tick;
//////  }

//////  if ((((Auto_spd.ID == 0) && (time_now >= 600)) || Auto_spd.ID == 2 || Auto_spd.ID == 4 || Auto_spd.ID == 6) && (fabs(Auto_spd.Sides_dis - Auto_spd.Target_side[Auto_spd.ID]) > dead_band))
//////  {
//////			if ((((Auto_spd.Turn_Tirg[Auto_spd.ID] - Auto_spd.Front_dis) >= 300) && (Auto_spd.Front_dis != 2000)) || (Auto_spd.Front_dis == 2000))
//////			{
//////					if (Auto_spd.Sides_dis != 2000)
//////					{
//////						Transverse_Correction(&Sides_PID, &Auto_spd);
//////					}
//////					else if ((Auto_spd.Sides_dis == 2000) && (Sides_cnt) >= 120)
//////					{
//////						Auto_spd.stage_yaw[Auto_spd.ID] = Auto_spd.std_stage_yaw[Auto_spd.ID];
//////						Sides_cnt = 0;
//////					}
//////					else
//////					{
//////						Sides_cnt++;
//////					}
//////			}
//////	}
//////  Set_Target_Yaw(Auto_spd.stage_yaw[Auto_spd.ID]);
//////  time_now += time_increment;
//////}

/*
ת��󲻷�����ʱ�䣨�Ӿ��ģ���
3.5 4.0 7.5
*/

// �Ƚ��ȶ��İ汾��22s����

////////#include "speed_race.h"
////////#include "dog.h"
////////#include "walk.h"
////////#include "imu.h"
////////#include "stm32h7xx.h"
////////#include "cmsis_os.h"
////////#include "walk.h"
////////#include "fsm.h"
////////#include "vision.h"
////////#include "pos_ctrl.h"
////////#include "delay_DWT.h"

////////extern int start_race_flag;
////////extern uint8_t change_step_length_flag; // �Ƿ�������������

////////#define offest 0
////////// ��д����ʱ�����߼�
////////speed_race_param Auto_spd = {
////////    // ����ش�����������ֵ
////////    .Front_dis = 2000,
////////    .Sides_dis = 2000,
////////    /*
////////      һ 610  490  365
////////      �� 520  420  320
////////      �� 40  -100 -180
////////         40  -90  -170
////////      �� 45  -70  -140
////////    */
////////    // �������
////////    .Max_left =    {610 + offest, 0, 580 + offest, 0,   40 + offest, 0,   45 + offest},
////////    .Target_side = {490 + offest, 0, 470 + offest, 0, -105 + offest, 0,  -70 + offest},
////////    .Max_right =   {365 + offest, 0, 340 + offest, 0, -175 + offest, 0, -190 + offest},

////////    .Turn_Tirg = {70 + offest, 100 + offest, 400 + offest},

////////    // �жϹ��ߵ��ĸ�λ��
////////    .ID = 0,
////////    .stage_dire = {Front, Back, Back, Front, Front, Back, Back},

////////    // �Զ���·��־λ,1Ϊ��ʼ
////////    .start_race_flag = 0,

////////    .stage_yaw =		 {0, -25, -45, -70, -89, -45, -43},
////////    .std_stage_yaw = {0, -25, -45, -70, -89, -45, -43},

////////};

////////PID_vision Sides_PID = {
////////    .Kp = 15.0f,
////////    .Kd = 0.05f,
////////    .k = 0.5f,
////////};

/////////**
//////// * @brief ;�иı�ƫ���ǽ��к�������
//////// * @param PD
//////// * @param Auto_spd
//////// */
////////void Transverse_Correction(PID_vision *PD, speed_race_param *Auto_spd)
////////{
////////  // �������error
////////  PD->err = (Auto_spd->Target_side[Auto_spd->ID] - Auto_spd->Sides_dis) / (Auto_spd->Max_left[Auto_spd->ID] - Auto_spd->Max_right[Auto_spd->ID]);

////////  // �ų�����Ӿ�����
////////  if (fabs(PD->err) >= 1.0f)
////////    PD->err = 0;

////////  // ���㲻��ȫD��
////////  PD->differ = PD->Kd * (1 - PD->k) * (PD->err - PD->last_err) + PD->k * PD->last_err;

////////  // �������
////////  float output = PD->Kp * PD->err + PD->differ;

////////  PD->last_err = PD->err;
////////  PD->differ = PD->last_differ;

////////  Auto_spd->stage_yaw[Auto_spd->ID] = output + Auto_spd->std_stage_yaw[Auto_spd->ID];
////////}

////////uint32_t time_now = 0; // ��λ����
////////uint32_t time_increment = 2;

////////float step_length_start = 0.02f;
////////float step_length_very_fast = 0.178f; // ֱ��ʱ���Ӳ�������
////////float step_length_fast = 0.165f;      // ֱ��ʱ���Ӳ�������
////////float step_length_slow = 0.105f;      // Ҫת��ʱ���ٲ��������������
////////float step_length_very_slow = 0.025f;

/////////*

////////float step_length_very_fast = 0.19f; // ֱ��ʱ���Ӳ�������
////////float step_length_fast = 0.18f;      // ֱ��ʱ���Ӳ�������
////////float step_length_slow = 0.12f;       // Ҫת��ʱ���ٲ��������������
////////float step_length_very_slow = 0.08f;
////////float step_length_very_fast = 0.15f; // ֱ��ʱ���Ӳ�������
////////float step_length_fast = 0.15f;      // ֱ��ʱ���Ӳ�������
////////float step_length_slow = 0.15f;      // Ҫת��ʱ���ٲ��������������
////////float step_length_very_slow = 0.08f;

////////float time1 = 2100; // ��һ��ֱ�߶����ÿ��ٲ�����ʱ��
////////float time2_start = 1000;
////////float time2 = 4000; // �ڶ���......
////////float time3 = 5000;
////////float time4 = 3500;

////////*/
////////float time_start = 400;
////////float time1 = 1800; // ��һ��ֱ�߶����ÿ��ٲ�����ʱ��
////////float time2_start = 400;
////////float time2 = 2000; // �ڶ���......
////////float time3_start = 400;
////////float time3 = 5500;
////////float time4_start = 400;
////////float time4 = 3500;

////////float time_stop_vision_1 = 1500;
////////float time_stop_vision_2 = 2500;
////////uint8_t stop_vision_flag = 0;
////////uint8_t clear_time_now = 1; // ��־λ����������time_now

////////float dead_band = 25;

////////void Stage_Judge(speed_race_param *Auto_spd)
////////{
////////  // ����ID:
////////  // ID = 0,������,����ʼλ���ߵ��ش���1
////////  // ID = 1,������,�ӱش���1ת45��
////////  // ID = 2,������,�ӱش���1�ߵ��ش���2
////////  // ID = 3,������,�ӱش���2ת45��
////////  // ID = 4,������,�ӱش���2�ߵ��ش���3
////////  // ID = 5,������,�ӱش���3ת45��
////////  // ID = 6,������,�ӱش���3�ߵ��ش���4
////////  // ID = 7,stop
////////  if (Auto_spd->ID == 0)
////////  {
////////		// ID = 0,������,����ʼλ���ߵ��ش���1
////////    Set_Walk_Length(step_length_very_fast);   
////////    Set_Turning_Kp(0.002f);
////////    Sides_PID.Kp = 15;
////////    Sides_PID.Kd = 0.1;
////////		if (time_now <= time_start)
////////      Set_Walk_Length(step_length_start);
////////		else if ((time_now >= time_start) && (time_now <= time1))
////////			Set_Walk_Length(step_length_very_fast);
////////    if (time_now >= time1)
////////      Set_Walk_Length(step_length_slow);
////////  }
////////	
////////	
////////  if ((Auto_spd->ID == 0) && ((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[0]) || (Auto_spd->Camera_dire_flag == -1)) && (Auto_spd->Front_dis > 0))
////////  {
////////    // ID = 1,������,�ӱش���1ת45��
////////    Set_Walk_Length(step_length_very_slow);
////////    Auto_spd->ID = 1;
////////    gait_param.VMC_Euler_Walk.last_differ_x = 0;
////////    gait_param.VMC_Euler_Walk.last_err_x = 0;
////////    Set_Turning_Kp(0.004f);
////////  }

////////	
////////  if ((Auto_spd->ID == 1) && (Euler_Angle.Yaw <= Auto_spd->std_stage_yaw[Auto_spd->ID]))
////////  {
////////    // ID = 2,������,�ӱش���1�ߵ��ش���2
////////    if (clear_time_now == 1)
////////    {
////////      time_now = 0;
////////      clear_time_now = 2;
////////    }
////////    Sides_PID.Kp = 15;
////////    Sides_PID.Kd = 0.1;

////////    Set_Walk_Length(step_length_slow);
////////    Auto_spd->ID = 2;
////////    Set_Turning_Kp(0.0025f); // 0.0007
////////  }
////////	if ((Auto_spd->ID == 2) && (time_now <= time2_start))
////////		Set_Walk_Length(step_length_very_slow);
////////  else if ((Auto_spd->ID == 2) && (time_now >= time2_start) && (time_now <= time2))
////////    Set_Walk_Length(step_length_very_fast);
////////  else if ((Auto_spd->ID == 2) && (time_now >= time2))
////////    Set_Walk_Length(step_length_slow);
////////	
////////	
////////  if ((Auto_spd->ID == 2) && ((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[1]) || (Auto_spd->Camera_dire_flag == 1)))
////////  {
////////    // ID = 3,������,�ӱش���2ת45��
////////    Set_Walk_Length(step_length_very_slow);
////////    Sides_PID.Kp = 15;
////////    Sides_PID.Kd = 0.1;
////////    Auto_spd->ID = 3;
////////    Set_Turning_Kp(0.0035f);
////////  }

////////	
////////  if ((Auto_spd->ID == 3) && (Euler_Angle.Yaw <= Auto_spd->std_stage_yaw[Auto_spd->ID]))
////////  {
////////    // ID = 4,������,�ӱش���2�ߵ��ش���3
////////    if (clear_time_now == 2)
////////    {
////////      time_now = 0;
////////      clear_time_now = 3;
////////    }
////////    Sides_PID.Kp = 15;  // 12
////////    Sides_PID.Kd = 0.1; // 0.5
////////    Auto_spd->ID = 4;
////////    Set_Walk_Length(step_length_very_fast);
////////    Set_Turning_Kp(0.0025f); // 0.0008
////////  }
////////	if ((Auto_spd->ID == 4) && (time_now <= time3_start))
////////    Set_Walk_Length(step_length_very_slow);
////////  else if ((Auto_spd->ID == 4) && (time_now >= time3_start) && (time_now <= time3))
////////    Set_Walk_Length(step_length_very_fast);
////////  else if ((Auto_spd->ID == 4) && (time_now >= time3))
////////    Set_Walk_Length(((step_length_slow + step_length_very_slow) / 2.0f));
////////	
////////	
////////  if ((Auto_spd->ID == 4) && (((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[2]) && (Auto_spd->Front_dis > 0)) || (Auto_spd->Camera_dire_flag == -1)))
////////  {
////////    // ID = 5,������,�ӱش���3ת45��
////////    Set_Walk_Length(step_length_very_slow);
////////    Sides_PID.Kp = 15;  // 12
////////    Sides_PID.Kd = 0.5; // 0.5
////////    Auto_spd->ID = 5;
////////    Set_Turning_Kp(0.0035f);
////////  }

////////	
////////  if ((Auto_spd->ID == 5) && (Euler_Angle.Yaw >= Auto_spd->std_stage_yaw[Auto_spd->ID]))
////////  {
////////    // ID = 6,������,�ӱش���3�ߵ��ش���4
////////    if (clear_time_now == 3)
////////    {
////////      time_now = 0;
////////      clear_time_now = 4;
////////    }
////////    Auto_spd->ID = 6;
////////    Set_Walk_Length(step_length_very_fast);
////////    Sides_PID.Kp = 15;
////////    Sides_PID.Kd = 0.1;
////////    Set_Turning_Kp(0.0025f);
////////  }	
////////	if ((Auto_spd->ID == 6) && (time_now <= time4_start))
////////    Set_Walk_Length(step_length_very_slow);
////////  else if ((Auto_spd->ID == 6) && (time_now >= time4_start) && (time_now <= time4))
////////    Set_Walk_Length(step_length_very_fast);
////////  else if ((Auto_spd->ID == 6) && (time_now >= time4))
////////    Set_Walk_Length(((step_length_slow + step_length_very_slow) / 2.0f));

////////	
////////  if ((Auto_spd->ID == 6) && (Auto_spd->Front_dis <= 100) && (Auto_spd->Front_dis > 0))
////////  {
////////    Auto_spd->ID = 7;
////////  }
////////}
////////int Sides_cnt = 0;
////////float race_timer = 0;
////////void start_race(void)
////////{
////////  Stage_Judge(&Auto_spd);
////////  Set_Walk_Dire(Auto_spd.stage_dire[Auto_spd.ID]);
////////  if ((leg[0].ctrl_state == Force_Ctrl) && (Auto_spd.ID == 0) && (GetNowDogState() != Stand) && (GetNowDogState() != Squat))
////////  {
////////    Change_Ctrl_State(Pos_Ctrl);
////////  }
////////  if ((Auto_spd.start_race_flag == 1) && (GetNowDogState() != Walk))
////////  {  
////////    Auto_spd.start_race_flag = 2;
////////    ChangeDogState(Walk);
////////    SetDogChangeAble(ENABLE);
////////    Auto_spd.ID = 0;
////////    clear_time_now = 1;
////////    Yaw_Reset(&Euler_Angle);
////////  }
////////  if ((Auto_spd.ID == 7) && (GetNowDogState() == Walk))
////////  {
////////    SetDogChangeAble(ENABLE);
////////    ChangeDogState(Keep);
////////		race_timer = Time_Points_Param.Now_Tick;
////////  }

////////  if ((((Auto_spd.ID == 0) && (time_now >= 600)) || Auto_spd.ID == 2 || Auto_spd.ID == 4 || Auto_spd.ID == 6) && (fabs(Auto_spd.Sides_dis - Auto_spd.Target_side[Auto_spd.ID]) > dead_band))
////////  {
////////			if ((((Auto_spd.Turn_Tirg[Auto_spd.ID] - Auto_spd.Front_dis) >= 300) && (Auto_spd.Front_dis != 2000)) || (Auto_spd.Front_dis == 2000))
////////			{
////////					if (Auto_spd.Sides_dis != 2000)
////////					{
////////						Transverse_Correction(&Sides_PID, &Auto_spd);
////////					}
////////					else if ((Auto_spd.Sides_dis == 2000) && (Sides_cnt) >= 120)
////////					{
////////						Auto_spd.stage_yaw[Auto_spd.ID] = Auto_spd.std_stage_yaw[Auto_spd.ID];
////////						Sides_cnt = 0;
////////					}
////////					else
////////					{
////////						Sides_cnt++;
////////					}
////////			}
////////	}
////////  Set_Target_Yaw(Auto_spd.stage_yaw[Auto_spd.ID]);
////////  time_now += time_increment;
////////}

// ǰ������İ汾����������

////////#include "speed_race.h"
////////#include "dog.h"
////////#include "walk.h"
////////#include "imu.h"
////////#include "stm32h7xx.h"
////////#include "cmsis_os.h"
////////#include "walk.h"
////////#include "fsm.h"
////////#include "vision.h"
////////#include "pos_ctrl.h"

////////extern int start_race_flag;
////////extern uint8_t change_step_length_flag; // �Ƿ�������������

////////#define offest 10
////////// ��д����ʱ�����߼�
////////speed_race_param Auto_spd = {
////////    // ����ش�����������ֵ
////////    .Front_dis = 2000,
////////    .Sides_dis = 2000,
////////	/*
////////		һ 610  490  365
////////		�� 520  420  320
////////		�� 40  -100 -180
////////			 40  -90  -170
////////		�� 45  -70  -140
////////	*/
////////		// �������
////////    .Max_left =	   {610 + offest, 0, 580 + offest, 0,  40 + offest,  0,  45 + offest},
////////		.Target_side = {490 + offest, 0, 480 + offest, 0, -125 + offest, 0, -70 + offest},
////////    .Max_right =   {365 + offest, 0, 380 + offest, 0, -175 + offest, 0, -190 + offest},

////////    .Turn_Tirg =   {70 + offest, 100 + offest, 550 + offest},

////////    // �жϹ��ߵ��ĸ�λ��
////////    .ID = 0,
////////    .stage_dire = {Front, Back, Back, Front, Front, Back, Back},

////////    // �Զ���·��־λ,1Ϊ��ʼ
////////    .start_race_flag = 0,

////////    .stage_yaw = 		 {0, -46, -45, -86, -90, -44, -45},
////////    .std_stage_yaw = {0, -46, -45, -86, -90, -44, -45},
////////};

////////PID_vision Sides_PID = {
////////    .Kp = 15.0f,
////////    .Kd = 0.05f,
////////    .k = 0.5f,
////////};

/////////**
//////// * @brief ;�иı�ƫ���ǽ��к�������
//////// * @param PD
//////// * @param Auto_spd
//////// */
////////void Transverse_Correction(PID_vision *PD, speed_race_param *Auto_spd)
////////{
////////  // �������error
////////  PD->err = (Auto_spd->Target_side[Auto_spd->ID] - Auto_spd->Sides_dis) / (Auto_spd->Max_left[Auto_spd->ID] - Auto_spd->Max_right[Auto_spd->ID]);

////////	// �ų�����Ӿ�����
////////	if (fabs(PD->err) >= 1.5f)
////////    PD->err = 0;

////////  // ���㲻��ȫD��
////////  PD->differ = PD->Kd * (1 - PD->k) * (PD->err - PD->last_err) + PD->k * PD->last_err;

////////  // �������
////////  float output = PD->Kp * PD->err + PD->differ;

////////  PD->last_err = PD->err;
////////  PD->differ = PD->last_differ;

////////  Auto_spd->stage_yaw[Auto_spd->ID] = output + Auto_spd->std_stage_yaw[Auto_spd->ID];
////////}

////////uint32_t time_now = 0; // ��λ����
////////uint32_t time_increment = 2;

////////float step_length_very_fast = 0.178f; // ֱ��ʱ���Ӳ�������
////////float step_length_slow = 0.105f;       // Ҫת��ʱ���ٲ��������������
////////float step_length_very_slow = 0.1f;

////////float time1 = 1800; // ��һ��ֱ�߶����ÿ��ٲ�����ʱ��
////////float time2_start = 1000;
////////float time2 = 3000; // �ڶ���......
////////float time3 = 5000;
////////float time4 = 3500;

////////float time_stop_vision_1 = 1500;
////////float time_stop_vision_2 = 2500;
////////uint8_t stop_vision_flag = 0;
////////uint8_t clear_time_now = 1; // ��־λ����������time_now

////////float dead_band = 25;

////////void Stage_Judge(speed_race_param *Auto_spd)
////////{
////////  // ����ID:
////////  // ID = 0,������,����ʼλ���ߵ��ش���1
////////  // ID = 1,������,�ӱش���1ת45��
////////  // ID = 2,������,�ӱش���1�ߵ��ش���2
////////  // ID = 3,������,�ӱش���2ת45��
////////  // ID = 4,������,�ӱش���2�ߵ��ش���3
////////  // ID = 5,������,�ӱش���3ת45��
////////  // ID = 6,������,�ӱش���3�ߵ��ش���4
////////  // ID = 7,stop
////////  if (Auto_spd->ID == 0)
////////  {
////////		Set_Walk_Length(step_length_very_fast);
////////		// ID = 0,������,����ʼλ���ߵ��ش���1
////////    Set_Turning_Kp(0.0025f);
////////    Sides_PID.Kp = 15;
////////    Sides_PID.Kd = 0.1;
////////    if (time_now >= time1)
////////      Set_Walk_Length(step_length_slow);
////////  }
////////  if ((Auto_spd->ID == 0) && ((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[0]) || (Auto_spd->Camera_dire_flag == -1)) && (Auto_spd->Front_dis > 0))
////////  {
////////		// ID = 1,������,�ӱش���1ת45��
////////    Auto_spd->ID = 1;
////////    gait_param.VMC_Euler_Walk.last_differ_x = 0;
////////    gait_param.VMC_Euler_Walk.last_err_x = 0;
////////    Set_Turning_Kp(0.004f);
////////  }

////////  if ((Auto_spd->ID == 1) && (Euler_Angle.Yaw <= Auto_spd->std_stage_yaw[Auto_spd->ID]))
////////  {
////////		// ID = 2,������,�ӱش���1�ߵ��ش���2
////////    if (clear_time_now == 1)
////////    {
////////      time_now = 0;
////////      clear_time_now = 2;
////////    }
////////    Sides_PID.Kp = 10;
////////    Sides_PID.Kd = 0.1;

////////    Set_Walk_Length(step_length_slow);
////////    Auto_spd->ID = 2;
////////    Set_Turning_Kp(0.0025f); // 0.0007
////////  }

////////  if ((Auto_spd->ID == 2) && (time_now >= time2_start))
////////    Set_Walk_Length(step_length_very_fast);
////////  if ((Auto_spd->ID == 2) && (time_now >= time2))
////////    Set_Walk_Length(step_length_slow);
////////  if ((Auto_spd->ID == 2) && ((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[1]) || (Auto_spd->Camera_dire_flag == 1)))
////////  {
////////		// ID = 3,������,�ӱش���2ת45��
////////    Sides_PID.Kp = 15;
////////    Sides_PID.Kd = 0.1;
////////    Auto_spd->ID = 3;
////////    Set_Turning_Kp(0.0025f);
////////  }

////////  if ((Auto_spd->ID == 3) && (Euler_Angle.Yaw <= Auto_spd->std_stage_yaw[Auto_spd->ID]))
////////  {
////////		// ID = 4,������,�ӱش���2�ߵ��ش���3
////////    if (clear_time_now == 2)
////////    {
////////      time_now = 0;
////////      clear_time_now = 3;
////////    }
////////    Sides_PID.Kp = 20;  // 12
////////    Sides_PID.Kd = 0.1; // 0.5
////////    Auto_spd->ID = 4;
////////    Set_Walk_Length(step_length_very_fast);
////////    Set_Turning_Kp(0.0025f); // 0.0008
////////  }
////////  if ((Auto_spd->ID == 4) && (time_now >= time3))
////////    Set_Walk_Length(step_length_slow);
////////  if ((Auto_spd->ID == 4) && (((Auto_spd->Front_dis <= Auto_spd->Turn_Tirg[2]) && (Auto_spd->Front_dis > 0)) || (Auto_spd->Camera_dire_flag == -1)))
////////  {
////////		// ID = 5,������,�ӱش���3ת45��
////////		Sides_PID.Kp = 10;  // 12
////////    Sides_PID.Kd = 0.5; // 0.5
////////    Auto_spd->ID = 5;
////////    Set_Turning_Kp(0.005f);
////////  }

////////  if ((Auto_spd->ID == 5) && (Euler_Angle.Yaw >= Auto_spd->std_stage_yaw[Auto_spd->ID]))
////////  {
////////		// ID = 6,������,�ӱش���3�ߵ��ش���4
////////    if (clear_time_now == 3)
////////    {
////////      time_now = 0;
////////      clear_time_now = 4;
////////    }
////////    Auto_spd->ID = 6;
////////    Set_Walk_Length(step_length_very_fast);
////////    Sides_PID.Kp = 15;
////////    Sides_PID.Kd = 0.1;
////////    Set_Turning_Kp(0.0025f);
////////  }
////////  if ((Auto_spd->ID == 6) && (time_now >= time4))
////////    Set_Walk_Length(step_length_slow);
////////  if ((Auto_spd->ID == 6) && (Auto_spd->Front_dis <= 100) && (Auto_spd->Front_dis > 0))
////////  {
////////    Auto_spd->ID = 7;
////////  }
////////}

////////void start_race(void)
////////{
////////  Stage_Judge(&Auto_spd);
////////  Set_Walk_Dire(Auto_spd.stage_dire[Auto_spd.ID]);
////////  if ((leg[0].ctrl_state == Force_Ctrl) && (Auto_spd.ID == 0) && (GetNowDogState() != Stand) && (GetNowDogState() != Squat))
////////  {
////////    Change_Ctrl_State(Pos_Ctrl);
////////  }
////////  if ((Auto_spd.start_race_flag == 1) && (GetNowDogState() != Walk))
////////  {
////////		Auto_spd.start_race_flag = 2;
////////    ChangeDogState(Walk);
////////    SetDogChangeAble(ENABLE);
////////    Auto_spd.ID = 0;
////////		clear_time_now = 1;
////////		Yaw_Reset(&Euler_Angle);

////////  }
////////  if ((Auto_spd.ID == 7) && (GetNowDogState() == Walk))
////////  {
////////    SetDogChangeAble(ENABLE);
////////    ChangeDogState(Keep);
////////  }

////////  if ((Auto_spd.ID == 0 || Auto_spd.ID == 2 || Auto_spd.ID == 4 || Auto_spd.ID == 6) && (fabs(Auto_spd.Sides_dis - Auto_spd.Target_side[Auto_spd.ID]) > dead_band))
////////  {
////////    if (
////////				(Auto_spd.Sides_dis != 2000) &&
////////				(
////////					(((Auto_spd.Turn_Tirg[Auto_spd.ID] - Auto_spd.Front_dis) >= 300) && (Auto_spd.Front_dis != 2000))
////////					|| (Auto_spd.Front_dis == 2000)
////////				)
////////		)
////////      Transverse_Correction(&Sides_PID, &Auto_spd);
////////    else
////////      Auto_spd.stage_yaw[Auto_spd.ID] = Auto_spd.std_stage_yaw[Auto_spd.ID];
////////  }

////////  Set_Target_Yaw(Auto_spd.stage_yaw[Auto_spd.ID]);
////////  time_now += time_increment;
////////}
