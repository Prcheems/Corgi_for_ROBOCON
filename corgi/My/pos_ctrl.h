#ifndef _POS_CTRL_H
#define _POS_CTRL_H

#include "stdint.h"
#include "dog.h"
#include "walk.h"
#include "action.h"
#include "imu.h"
#include "jump.h"

void line_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float T, float t);
void Cycloid_Pos(uint8_t legid, float start_x, float start_z, float end_x, float max_z, float T, float t);
void Bezier_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t);
void Bezier_Stance_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t);
void gait_posctrl(uint8_t legid, GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t);
void Walk_Away_Posctrl(GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t);
void Stand_Up_Posctrl(float t, float tm, float *h);
void Squat_Down_Posctrl(float t, float tm, float *x, float *z);
void Keep_All_Leg_Posctrl(float X0, float X1, float X2, float X3, float Z0, float Z1, float Z2, float Z3);
bool Jump_s_pos(uint8_t legid, JumpParam *Jump_Param, action_param *Action_Param, Euler *Euler_Angle, float t);
void Change_Ctrl_State(Ctrl_State state);
Ctrl_State Get_Now_Ctrl_State(void);

extern Pos_motor prep_param;
extern Pos_motor jump_param;
extern Pos_motor luanch_param;
#endif
