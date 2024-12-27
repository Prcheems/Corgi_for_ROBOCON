#ifndef BACKFLIP_H
#define BACKFLIP_H

#include "dog.h"
#include "stdbool.h"
#include "action.h"
#include "imu.h"

typedef struct
{
    float Prep_Rho[4];         // 蓄力时关节电机到足端的距离
    float Prep_Theta[4];       // 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
    float Prep_Time[4];        // 蓄力时收腿动作用的时间
    float Prep_Buffer_Time[4]; // 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
                               // 蓄力时用的总时间是Prep_Time+Prep_Buffer_Time

    float Backflip_1_Rho[4];     		  // 第一次起跳时关节电机到足端的距离
    float Backflip_1_Theta[4];   		  // 第一次起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
    float Backflip_1_Time[4];      // 第一次起跳时蹬腿动作用的时间

    float Shift_Rho[4];            // 第一次起跳后后腿在空中摆动到与身体成一条线时关节电机到足端的距离
    float Shift_Theta[4];          // 第一次起跳后后腿在空中摆动到与身体成一条线时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
    float Shift_Time[4];           // 第一次起跳后后腿在空中摆动到与身体成一条线用的时间

    float Backflip_2_Rho[4];   // 第二次起跳时关节电机到足端的距离
    float Backflip_2_Theta[4]; // 第二次起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
    float Backflip_2_Time[4];  // 第二次起跳时蹬腿动作用的时间

    float Load_Rho[4];          // 着陆时关节电机到足端的距离
    float Load_Theta[4];        // 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
    float Swing_in_Air_Time[4]; // 第二次起跳完成后后腿的回摆时间,腿摆回去以后就落地了
    float Land_Buffer_Time[4];  // 落地缓冲时间

    float Std_Prep_Theta[4];       // 标准角度,以01腿为基准的角度,用来适配手柄和方便给参数用的
    float Std_Backflip_1_Theta[4]; // 标准角度,以01腿为基准的角度,用来适配手柄和方便给参数用的
    float Std_Backflip_2_Theta[4]; // 标准角度,以01腿为基准的角度,用来适配手柄和方便给参数用的
    float Std_Shift_Theta[4];      // 标准角度,以01腿为基准的角度,用来适配手柄和方便给参数用的
    float Std_Load_Theta[4];       // 标准角度,以01腿为基准的角度,用来适配手柄和方便给参数用的

} BackflipParam;

typedef struct
{
    PD VMC_Swing[4];   // 摆动参数
    PD VMC_Cushion[4]; // 落地缓冲参数
} Backflip_VMC_Param;

typedef enum
{
    First_Flip,		// 第一次跳跃,第一次前空翻
		Second_Flip,	// 第二次跳跃,第二次前空翻
} BackflipType;

bool Backflip_s(uint8_t legid, BackflipParam *Backflip_Param, action_param *Action_Param, Euler *Euler_Angle, float t);
extern BackflipParam Backflip_Param[2];
extern BackflipType Backfliptype;

#endif

