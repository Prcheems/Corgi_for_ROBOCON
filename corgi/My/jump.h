#ifndef JUMP_H
#define JUMP_H

#include "dog.h"
#include "stdbool.h"
#include "action.h"
#include "imu.h"
#include "A1_motor.h"

typedef struct
{
    float Prep_Rho[4];         // 蓄力时关节电机到足端的距离
    float Prep_Theta[4];       // 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
    float Prep_Time[4];        // 蓄力时收腿动作用的时间
    float Prep_Buffer_Time[4]; // 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
                               // 蓄力时用的总时间是Prep_Time+Prep_Buffer_Time

    float Jump_Rho[4];         // 起跳时关节电机到足端的距离
    float Jump_Theta[4];       // 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
    float Jump_Time[4];        // 起跳时蹬腿动作用的时间
    float Jump_Buffer_Time[4]; // 起跳时蹬腿完成后的缓冲时间
                               // 起跳用的总时间是Jump_Time+Jump_Buffer_Time

    float Tuck_Rho[4];          // 起跳后在空中收腿时关节电机到足端的距离
    float Tuck_Theta[4];        // 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
    float Tuck_Time[4];         // 起跳后在空中收腿用的时间
    float Tuck_Buffer_Time[4];  // 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
    float Swing_in_Air_Time[4]; // 收腿完成后腿的回摆时间,腿摆回去以后就落地了

    float Load_Rho[4];         // 着陆时关节电机到足端的距离
    float Load_Theta[4];       // 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
    float Land_Buffer_Time[4]; // 落地缓冲时间

    float Std_Prep_Theta; // 标准角度,以01腿为基准的角度,用来适配手柄和方便给参数用的
    float Std_Jump_Theta; // 标准角度,以01腿为基准的角度,用来适配手柄和方便给参数用的
    float Std_Tuck_Theta; // 标准角度,以01腿为基准的角度,用来适配手柄和方便给参数用的
    float Std_Load_Theta; // 标准角度,以01腿为基准的角度,用来适配手柄和方便给参数用的

} JumpParam;

typedef struct
{
    PD VMC_Swing[4];   // 摆动参数
    PD VMC_Cushion[4]; // 落地缓冲参数
		float Preset_Kp_x[4];
		float Preset_Kp_z[4];
		float Boost_x;
		float Boost_z;
} Jump_VMC_Param;

typedef enum
{
	  First_Step = 0,   // 
    Second_Step,  		// 
    Third_Step,  			// 
    Bridge,
		Bridge2,	// 
    GroundForward,    // 平地向前跳跃
    GroundBack,       // 平地向后跳跃
    GroundLeft,       // 原地左转跳跃
    GroundRight,      // 原地右转跳跃
    GroundUpToStep,   // 平地到台阶

    StepDownToGround, // 下台阶到平地
    ClimbUp,          // 上坡
    ClimbDown,        // 下坡
    JumpTest1,        // 兔子跳
    JumpTest2,        // 前腿蹦 (用于跨栏)
    JumpTest3,        // 后腿蹦 (用于跨栏)
} JumpType;

bool Jump_s(uint8_t legid, JumpParam *Jump_Param, action_param *Action_Param, Euler *Euler_Angle, float t);
extern JumpParam Jump_Param[15];
extern Jump_VMC_Param Jump_VMC;
#endif
