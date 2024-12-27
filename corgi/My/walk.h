#ifndef _WALK_H
#define _WALK_H
#include "stdint.h"
#include "dog.h"
#include "action.h"

typedef enum
{
    Stance = 0,
    Swing = 1,
} leg_state;

typedef enum
{
    First = 0,
    Later = 1,
    Stop = 2,
} walk_state;

typedef enum
{
    Front = 0,
    Back = 1,
} dire_state;
/*
typedef enum
{
    Stay = 0,
    Forward,
    Back,
    Left,
    Right
} GaitDirection; // ���߷���
*/
typedef enum
{
    Gait_None = 0,
    Trot,
    LeftTrot,         // ԭ����ת
    RightTrot,        // ԭ����ת
    LeftForwardTrot,  // ǰ����ת
    RightForwardTrot, // ǰ����ת
    LeftBackTrot,     // ������ת
    RightBackTrot,    // ������ת
    StepUpMove,
    LowTrot, // ����̬,���ߡ�������С
    UpHill,
    HighTrot, // ��˫ľ����
    UpStep,   // ��̨����
} Gait;       // ��̬

typedef enum
{
    None = 0,  // ����б��
    Upslope,   // ����
    Downslope, // ����
} scope_state; // ��б��״̬

typedef struct
{
    float Axis_Offset; // x��������ƫ����
    float Std_Height;  // ��б��ʱ��Сվ���߶�
} slope;
typedef struct
{
    leg_state Leg_State[4]; // �ȵ�״̬(֧��/�ڶ���)
    walk_state Walk_State;  // walk״̬(��/�˶�/ֹͣ)
    dire_state Dire;        // ��ǰ����������
    float stance_height;    // վ���߶� (m)
    float step_length[4];   // ���� (m)
    float std_step_length;  // ��׼����,���㸳ֵ(m)
    float step_height;      // ���� (m)
    float step_height_down; // ֧�����³�����
    float swing_percent;    // �ڶ���ٷֱ�,ָһ���ڶ������ڰڶ���ʱ��
    float term;             // ���� (s)

    uint32_t Cross_Index[4]; // ÿ������������
    float t;                 // ��ǰʱ��
    float last_t;            // �ݴ��ϴε�ʱ��,����֧�Űڶ����л�

    PD VMC_Swing_Walk[4];  // VMC�ڶ������
    PD VMC_Stance_Walk[4]; // VMC֧�������
    PD VMC_Euler_Walk;     // VMC��̬���Ʋ���
    float Yaw_max;         // ƫ���ǽ�����󲹳�����

    Pos_motor Posctrl_Swing;  // Swing���λ�ز���
    Pos_motor Posctrl_Stance; // Stance���λ�ز���

    slope Scope;
} GaitParam;

extern GaitParam gait_param;
void Set_Target_Yaw(float Target_Yaw);
void Set_Walk_Dire(dire_state Dire);
void Set_Walk_Length(float Walk_Length);
void Pitch_Pos_Offest(Euler *Euler, GaitParam *gait_param);
void Yaw_Pos_Crol(Euler *Euler, GaitParam *gait_param, float Max_Changable_Step_Length);
void Set_Turning_Kp(float Kp);

void Walk_Away_Posctrl(GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t);
#endif
