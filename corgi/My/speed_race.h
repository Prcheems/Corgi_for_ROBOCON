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
    float k;  // 不完全微分系数

    float err;         // 当前误差
    float last_err;    // 上一时刻误差
    float differ;      // 当前微分项大小
    float last_differ; // 上一时刻微分项大小
} PID_vision;

extern PID_vision Sides_PID;
extern speed_race_param Auto_spd;

#endif
