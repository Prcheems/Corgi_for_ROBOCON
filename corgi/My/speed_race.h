#ifndef _SPEED_RACE_H
#define _SPEED_RACE_H

#include "walk.h"
typedef struct
{
    float Sides_dis;
    float Front_dis;
		float Camera_dire_flag;
	
    float Max_right[9];
    float Max_left[7];
    float Turn_Tirg[3];
    float Target_side[9];

    float std_stage_yaw[7];
		float stage_yaw[7];
    dire_state stage_dire[7];
    int start_race_flag;

    int ID;
} speed_race_param;

typedef struct
{
    float Kp; // Kp
    float Kd; // Kd
    float k;  // ����ȫ΢��ϵ��

    float err;         // ��ǰ���
    float last_err;    // ��һʱ�����
    float differ;      // ��ǰ΢�����С
    float last_differ; // ��һʱ��΢�����С
} PID_vision;

extern PID_vision Sides_PID;
extern speed_race_param Auto_spd;

#endif
