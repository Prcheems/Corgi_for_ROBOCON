#ifndef _ACTION_H
#define _ACTION_H

#include "dog.h"
#include "stdbool.h"
#include "math.h"
typedef enum
{
  Launch = 0,
  Run = 1,
  Finish = 2,
} action_state;

typedef struct
{
  // ȫ״̬ͨ�ò���
  action_state Launch_Flag;
  Point Init_Pos[4];

  // Keep״̬����
  float Max_Diff;
  PD VMC_Stance_Keep[4];  // Keep״̬VMC֧�������
  Pos_motor Posctrl_keep; // Keep״̬���λ�ز���
	uint8_t keep_type;      // Keep����
	
  // Stand״̬����
  float stand_height[4];
  float stand_tm;
  PD VMC_Swing_Stand[4];   // Stand״̬VMC�ڶ������
  Pos_motor Posctrl_stand; // stand״̬���λ�ز���

  // Squat״̬����;
  float squat_x[4];
  float squat_z[4];
  float squat_tm;
  PD VMC_Swing_Squat[4];   // Squat״̬VMC�ڶ������
  Pos_motor Posctrl_Squat; // Squat״̬���λ�ز���

} action_param;

extern action_param Action_Param;

void Stand_Up(float t, float tm, float *h);
void Squat_Down(float t, float tm, float *x, float *z);
bool Abs_Diff_Judge(float a, float b, float Max_Diff);
bool Judge_Keep_State(void);
void Keep_All_Leg(float X0, float X1, float X2, float X3, float Z0, float Z1, float Z2, float Z3, PD *Stance);
void Keep_One_Leg(uint8_t legid, float X, float Z, PD *Stance);

#endif
