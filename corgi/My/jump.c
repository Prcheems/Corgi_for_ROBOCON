#include "jump.h"
#include "A1_motor.h"
#include "timer.h"
#include "cmsis_os.h"
#include "walk.h"
#include "timer.h"
#include "delay_DWT.h"
#include "pos_ctrl.h"

static float tick_start = 0; // 动作的开始时间(s)

JumpParam Jump_Param[15];
JumpType Jumptype = First_Step;
Jump_VMC_Param Jump_VMC =
    {
        //        .Preset_Kp_x = {4000, 4000, 4000, 4000},
        //        .Preset_Kp_z = {5000, 5000, 5000, 5000},
        .VMC_Cushion = {{2000, 3500, 0.5, 2500, 5000, 0.5, 0, 0},
                        {2000, 3500, 0.5, 2500, 5000, 0.5, 0, 0},
                        {2000, 3500, 0.5, 2500, 5000, 0.5, 0, 0},
                        {2000, 3500, 0.5, 2500, 5000, 0.5, 0, 0}},

        .VMC_Swing = {{5000, 1300, 0.5, 9000, 900, 0.5, 0, 0},
                      {5000, 1300, 0.5, 9000, 900, 0.5, 0, 0},
                      {5000, 1300, 0.5, 9000, 900, 0.5, 0, 0},
                      {5000, 1300, 0.5, 9000, 900, 0.5, 0, 0}},
        // x方向Kp x方向Kd x方向不完全微分系数
        // z方向Kp z方向Kd z方向不完全微分系数
        // 前馈x 前馈z

        //        .Boost_x = -91,
        //        .Boost_z = 250,

        .Boost_x = 0,
        .Boost_z = 0,
};

/* --------------------------------------跳跃--------------------------------------------- */

///////**
////// * @brief 把陀螺仪的反馈Pitch叠加到角度上提高稳定度,上坡时候Pitch是正的
////// * @param Euler 欧拉角结构体,用于读取俯仰角
////// * @param Jump_Param 跳跃结构体
////// * @warning 这个Load_Theta还没有想到很好的办法规避浮动值,先这么给试一下
////// */
//////void Pitch_Jump_Offest(Euler *Euler, JumpParam *Jump_Param)
//////{
//////        Jump_Param->Prep_Theta[0] = Jump_Param->Std_Prep_Theta + Euler->Pitch;
//////        Jump_Param->Prep_Theta[1] = Jump_Param->Std_Prep_Theta + Euler->Pitch;
//////        Jump_Param->Prep_Theta[2] = 180.0f - Jump_Param->Std_Prep_Theta + Euler->Pitch;
//////        Jump_Param->Prep_Theta[3] = 180.0f - Jump_Param->Std_Prep_Theta + Euler->Pitch;

//////        Jump_Param->Jump_Theta[0] = Jump_Param->Std_Jump_Theta + Euler->Pitch;
//////        Jump_Param->Jump_Theta[1] = Jump_Param->Std_Jump_Theta + Euler->Pitch;
//////        Jump_Param->Jump_Theta[2] = 180.0f - Jump_Param->Std_Jump_Theta + Euler->Pitch;
//////        Jump_Param->Jump_Theta[3] = 180.0f - Jump_Param->Std_Jump_Theta + Euler->Pitch;

//////        Jump_Param->Tuck_Theta[0] = Jump_Param->Std_Tuck_Theta + Euler->Pitch;
//////        Jump_Param->Tuck_Theta[1] = Jump_Param->Std_Tuck_Theta + Euler->Pitch;
//////        Jump_Param->Tuck_Theta[2] = 180.0f - Jump_Param->Std_Tuck_Theta + Euler->Pitch;
//////        Jump_Param->Tuck_Theta[3] = 180.0f - Jump_Param->Std_Tuck_Theta + Euler->Pitch;

//////        // 这个Load_Theta还没有想到很好的办法规避浮动值,先这么给试一下
//////        Jump_Param->Load_Theta[0] = Jump_Param->Std_Load_Theta + Euler->Pitch;
//////        Jump_Param->Load_Theta[1] = Jump_Param->Std_Load_Theta + Euler->Pitch;
//////        Jump_Param->Load_Theta[2] = 180.0f - Jump_Param->Std_Load_Theta + Euler->Pitch;
//////        Jump_Param->Load_Theta[3] = 180.0f - Jump_Param->Std_Load_Theta + Euler->Pitch;
//////    }
//////}

/**
 * @brief
 * @param
 * @param
 * @warning
 */
void Pitch_Jump_Offest(Euler *Euler, JumpParam *Jump_Param)
{
    Jump_Param->Prep_Theta[0] = Jump_Param->Std_Prep_Theta;
    Jump_Param->Prep_Theta[1] = Jump_Param->Std_Prep_Theta;
    Jump_Param->Prep_Theta[2] = 180.0f - Jump_Param->Std_Prep_Theta;
    Jump_Param->Prep_Theta[3] = 180.0f - Jump_Param->Std_Prep_Theta;

    Jump_Param->Jump_Theta[0] = Jump_Param->Std_Jump_Theta;
    Jump_Param->Jump_Theta[1] = Jump_Param->Std_Jump_Theta;
    Jump_Param->Jump_Theta[2] = 180.0f - Jump_Param->Std_Jump_Theta;
    Jump_Param->Jump_Theta[3] = 180.0f - Jump_Param->Std_Jump_Theta;

    Jump_Param->Tuck_Theta[0] = Jump_Param->Std_Tuck_Theta;
    Jump_Param->Tuck_Theta[1] = Jump_Param->Std_Tuck_Theta;
    Jump_Param->Tuck_Theta[2] = 180.0f - Jump_Param->Std_Tuck_Theta;
    Jump_Param->Tuck_Theta[3] = 180.0f - Jump_Param->Std_Tuck_Theta;

    // 这个Load_Theta还没有想到很好的办法规避浮动值,先这么给试一下
    Jump_Param->Load_Theta[0] = Jump_Param->Std_Load_Theta;
    Jump_Param->Load_Theta[1] = Jump_Param->Std_Load_Theta;
    Jump_Param->Load_Theta[2] = 180.0f - Jump_Param->Std_Load_Theta;
    Jump_Param->Load_Theta[3] = 180.0f - Jump_Param->Std_Load_Theta;
}

/**
 * @brief 给前馈力,让狗跳远点
 * @param legid
 * @param Jump_Param
 * @param start_time 加前馈力的开始时间
 * @param end_time 加前馈力的结束时间
 * @param t 当前时间
 */
void Jump_Boost(uint8_t legid, JumpParam *Jump_Param, float start_time, float end_time, float t)
{
    if ((t >= start_time) && (t <= end_time))
    {
        if ((leg[legid].Foot.p.x * leg[legid].Foot.p.x + leg[legid].Foot.p.z * leg[legid].Foot.p.z) <= (Jump_Param->Jump_Rho[legid] - 0.05f))
        {
            Jump_VMC.VMC_Swing[legid].Feedforward_z = Jump_VMC.Boost_z;
            Jump_VMC.VMC_Swing[legid].Feedforward_x = Jump_VMC.Boost_x;
            Jump_VMC.VMC_Swing[legid].Kp_x = 0;
            Jump_VMC.VMC_Swing[legid].Kp_z = 0;
        }
    }
    else
    {
        Jump_VMC.VMC_Swing[legid].Feedforward_z = 0;
        Jump_VMC.VMC_Swing[legid].Feedforward_x = 0;
        Jump_VMC.VMC_Swing[legid].Kp_x = Jump_VMC.Preset_Kp_x[legid];
        Jump_VMC.VMC_Swing[legid].Kp_z = Jump_VMC.Preset_Kp_z[legid];
    }
}

/* ---------------------------------------跳跃过程函数----------------------------------------- */

/*
t0 + Prep_Time + Prep_Buffer_Time + Jump_Time + Jump_Buffer_Time + Tuck_Time + Tuck_Buffer_Time + Swing_in_Air_Time + Land_Buffer_Time
    准备过程中收腿    收腿以后缓冲    跳跃蹬腿时间   蹬腿之后缓冲保持     收腿时间    收腿以后缓冲保持
---------------t1-----------------
---------------------------------t2------------------------------
-------------------------------------------------t3----------------------------------------------------------------
----------------------------------------------------------t4--------------------------------------------------------------------------

t0 ~ (t0+Prep_Time) ~
t1 = (t0+Prep_Time+Prep_Buffer_Time) ~ (t1+Jump_Time) ~
t2 = (t1+Jump_Time+Jump_Buffer_Time) ~ (t2+Tuck_Time) ~ (t2+Tuck_Time+Tuck_Buffer_Time) ~
t3 = (t2+Tuck_Time+Tuck_Buffer_Time+Swing_in_Air_Time) ~ (t3+Land_Buffer_Time)

*/

bool Jump_s(uint8_t legid, JumpParam *Jump_Param, action_param *Action_Param, Euler *Euler_Angle, float t)
{
    bool Jump_State = false;
    if (legid <= 3 && legid >= 0) // 严防数组越界
    {
        // 时间安排
        float t0 = 0;
        float t1 = t0 + Jump_Param->Prep_Time[legid] + Jump_Param->Prep_Buffer_Time[legid];
        float t2 = t1 + Jump_Param->Jump_Time[legid] + Jump_Param->Jump_Buffer_Time[legid];
        float t3 = t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid] + Jump_Param->Swing_in_Air_Time[legid];
        float t4 = t3 + Jump_Param->Land_Buffer_Time[legid];

        //        Jump_Boost(legid, Jump_Param, t1, t1 + Jump_Param->Jump_Time[legid], t);

        // 跳跃过程Start!
        if (t <= t0) // 什么都不做
        {
            // 0 ~ t0
            // line函数在动作未开始时在原地停着因此运动始末点均为同一点,即足端初始位置点
            // 停留时间t0,采用VMC_Swing参数
            line(legid,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 t0, t, &Jump_VMC.VMC_Swing[legid]);
        }
        else if ((t > t0) && (t <= t1)) // 收腿准备起跳
        {
            if (t <= (t0 + Jump_Param->Prep_Time[legid]))
            {
                // t0 ~ (t0+Prep_Time)
                // line函数从Init_Pos到收腿目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
                // 运动时间Prep_Time,采用VMC_Swing参数
                line(legid,
                     Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                     Jump_Param->Prep_Rho[legid] * cosf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * sinf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Time[legid], t - t0, &Jump_VMC.VMC_Swing[legid]);
            }
            else if (t > (t0 + Jump_Param->Prep_Time[legid]))
            {
                // (t0+Prep_Time) ~ t1
                // line函数在收腿目标位置稍作停留,使身体稳定,因此运动始末点均为同一点,即预设起跳点
                // 停留时间Prep_Buffer_Time,采用VMC_Swing参数
                line(legid,
                     Jump_Param->Prep_Rho[legid] * cosf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * sinf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * cosf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * sinf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Buffer_Time[legid], t - t0 - Jump_Param->Prep_Time[legid], &Jump_VMC.VMC_Swing[legid]);
            }
        }
        else if ((t > t1) && (t <= t2)) // 起跳
        {
            if (t <= (t1 + Jump_Param->Jump_Time[legid]))
            {
                // t1 ~ (t1+Jump_Time)
                // line函数从收腿位置到蹬腿起跳目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
                // 运动时间Jump_Time,采用VMC_Swing参数
                line(legid,
                     Jump_Param->Prep_Rho[legid] * cosf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * sinf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Time[legid], t - t1, &Jump_VMC.VMC_Swing[legid]);
            }
            else if (t > (t1 + Jump_Param->Jump_Time[legid]))
            {
                // (t1+Jump_Time) ~ (t1+Jump_Time + Jump_Buffer_Time)
                // line函数在蹬腿目标位置稍作停留,因此运动始末点均为同一点,即预设起跳点
                // 停留时间Jump_Buffer_Time,采用VMC_Swing参数
                line(legid,
                     Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Buffer_Time[legid], t - t1 - Jump_Param->Jump_Time[legid], &Jump_VMC.VMC_Swing[legid]);
            }
        }
        else if ((t > t2) && (t <= t3)) // 空中摆腿
        {
            if (t <= (t2 + Jump_Param->Tuck_Time[legid])) // 空中收腿
            {
                // t2 ~ (t2+Tuck_Time)
                // line函数从蹬腿起跳位置到空中收腿目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
                // 运动时间Tuck_Time,采用VMC_Swing参数
                line(legid,
                     Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Time[legid], t - t2, &Jump_VMC.VMC_Swing[legid]);
            }
            else if ((t > (t2 + Jump_Param->Tuck_Time[legid])) && (t <= (t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid])))
            {
                // (t2+Tuck_Time) ~ (t2+Tuck_Time+Tuck_Buffer_Time)
                // line函数在收腿目标位置稍作停留,因此运动始末点均为同一点
                // 停留时间Tuck_Buffer_Time,采用VMC_Swing参数
                line(legid,
                     Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Buffer_Time[legid], t - t2 - Jump_Param->Tuck_Time[legid], &Jump_VMC.VMC_Swing[legid]);
            }
            else if (t > (t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid])) // 准备着陆
            {
                // (t2+Tuck_Time+Tuck_Buffer_Time) ~ t3
                // line函数从空中收腿位置到落地时预设目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
                // 运动时间Swing_in_Air_Time,采用VMC_Swing参数
                line(legid,
                     Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Load_Rho[legid] * cosf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Load_Rho[legid] * sinf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Swing_in_Air_Time[legid],
                     t - t2 - Jump_Param->Tuck_Time[legid] - Jump_Param->Tuck_Buffer_Time[legid], &Jump_VMC.VMC_Swing[legid]);
            }
        }
        else if (t > t3 && t <= t4) // 阻尼减震
        {
            Keep_One_Leg(legid,
                         Jump_Param->Load_Rho[legid] * cosf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Load_Rho[legid] * sinf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN), Jump_VMC.VMC_Cushion);
        }

        else if (t > t4)
        {
            Jump_State = true;

            line(legid,
                 Jump_Param->Load_Rho[legid] * cosf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                 Jump_Param->Load_Rho[legid] * sinf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                 Jump_Param->Load_Rho[legid] * cosf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                 Jump_Param->Load_Rho[legid] * sinf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                 1, 1, &Jump_VMC.VMC_Swing[legid]);
        }
    }
    return Jump_State;
}

/* ---------------------------------------状态机----------------------------------------- */

// 更新跳跃类型
void ChangeJumpType(JumpType x)
{
    Jumptype = x;
}

void jump_enter()
{
    SetDogChangeAble(DISABLE); // 动作完成前禁止切换状态
    ChangeJumpType(Jumptype);  // 更新跳跃状态
    GetTick();
    Change_Dog_Pitch_State(Follow_Ground);
    tick_start = Time_Points_Param.Now_Tick; // 获取当前时间
    for (int i = 0; i <= 3; i++)
    {
        Get_Now_Pos_Spd(i);
        Euler_Update(&Euler_Angle);
        Coordinate_Transform(&leg[i].Foot.p, &Action_Param.Init_Pos[i], -Euler_Angle.Pitch, &leg[i], i);
    }
		if (Get_Now_Ctrl_State() == Pos_Ctrl)
    {
			  Set_All_Motor_Param(&jump_param);
		}
}

void jump_run()
{
    GetTick();
    float t = Time_Points_Param.Now_Tick - tick_start;
		Change_Ctrl_State(Pos_Ctrl);

    // 俯仰角修正
    //    Pitch_Jump_Offest(&Euler_Angle, &Jump_Param[Jumptype]);

    bool finish_state = false;
    if (Get_Now_Ctrl_State() == Force_Ctrl)
    {
        finish_state = Jump_s(0, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s(1, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s(2, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s(3, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);

        if (finish_state == true)
            SetDogChangeAble(ENABLE); // 所有腿的跳跃动作结束后允许跳出
    }
    else if (Get_Now_Ctrl_State() == Pos_Ctrl)
    {
			  Set_All_Motor_Param(&gait_param.Posctrl_Stance);
        finish_state = Jump_s_pos(0, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s_pos(1, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s_pos(2, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s_pos(3, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);

        if (finish_state == true)
            SetDogChangeAble(ENABLE); // 所有腿的跳跃动作结束后允许跳出
    }
}
/* -----------------------------------参数初始化------------------------------------------ */

JumpParam Jump_Param[15] = {
    {
        // First_Step 一起起跳

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.155, 0.155, 0.155, 0.155},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {120, 120, 60, 60},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.395, 0.395, 0.395, 0.395},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.4, 0.4, 0.4, 0.4},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {120, 120, 60, 60},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.121, 0.121, 0.121, 0.121},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0, 0, 0, 0},
				
        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.16, 0.16, 0.16, 0.16},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {89, 89, 91, 91},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.3, 0.3, 0.3, 0.3},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        // 这个时间不得大于0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; 着陆时关节电机到足端的距离
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正 向的夹角0,逆时针为正
        {89, 89, 91, 91},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,
    },
    {
        // 腿长最短0.14667m 最长0.4048m

        // Second_Step 第二段台阶跳跃

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.155, 0.155, 0.155, 0.155},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {91, 120, 60, 89},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.3, 0.45, 0.45, 0.3},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.40, 0.40, 0.40, 0.40},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {91, 120, 60, 89},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.05, 0.12, 0.12, 0.05},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0, 0, 0, 0},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.16, 0.16, 0.16, 0.16},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {76, 76, 104, 104},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.3, 0.3, 0.3, 0.3},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        // 这个时间不得大于0.15s
        {0.1, 0.1, 0.1, 0.1},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; 着陆时关节电机到足端的距离
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {75, 75, 105, 105},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,

    },
    {
        // Third_Step 第三段台阶跳跃

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.18, 0.18, 0.18, 0.18},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {91, 110, 70, 89},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.3, 0.5, 0.5, 0.3},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {91, 110, 70, 89},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.15, 0.15, 0.15, 0.15},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0, 0, 0, 0},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.16, 0.16, 0.16, 0.16},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {76, 76, 104, 104},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        // 这个时间不得大于0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; 着陆时关节电机到足端的距离
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {90, 90, 90, 90},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,
    },
    {
        // 腿长最短0.14667m 最长0.4048m

        // Bridge 双木桥跳跃

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.2, 0.2, 0.2, 0.2},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {91, 75, 105, 89},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.3, 0.4, 0.4, 0.3},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.4, 0.2, 0.2, 0.4},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {90, 105, 75, 90},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.15, 0.15, 0.15, 0.15},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0, 0, 0, 0},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.2, 0.2, 0.2, 0.2},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {50, 105, 75, 130},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        // 这个时间不得大于0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; 着陆时关节电机到足端的距离
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {50, 105, 75, 130},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,

    },
    {
        // 腿长最短0.14667m 最长0.4048m

        // Bridge2 双木桥跳跃后腿

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.2, 0.18, 0.18, 0.2},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {91, 91, 89, 89},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.3, 0.4, 0.4, 0.3},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.2, 0.39, 0.39, 0.2},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {91, 110, 70, 89},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.15, 0.15, 0.15, 0.15},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0, 0, 0, 0},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.2, 0.39, 0.39, 0.2},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {90, 60, 120, 90},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        // 这个时间不得大于0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; 着陆时关节电机到足端的距离
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {90, 60, 120, 90},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,

    },
    {
        // Ground_Forward 向前跳

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.40, 0.38, 0.37, 0.37},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.15, 0.15, 0.15, 0.15},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0, 0, 0, 0},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.16, 0.16, 0.16, 0.16},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {76, 76, 104, 104},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        // 这个时间不得大于0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; 着陆时关节电机到足端的距离
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {90, 90, 90, 90},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,
    },
    {
        // GroundBack 平地向后跳跃

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.39, 0.39, 0.39, 0.39},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.15, 0.15, 0.15, 0.15},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0, 0, 0, 0},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.16, 0.16, 0.16, 0.16},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {76, 76, 104, 104},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        // 这个时间不得大于0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; 着陆时关节电机到足端的距离
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {90, 90, 90, 90},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,
    },
    /* 下面的没有改结构体初始值,想用就先改一下
    {
        // GroundBack 平地向后跳跃

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {75, 75, 105, 105},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.27, 0.27, 0.27, 0.27},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.59, 0.59, 0.59, 0.59},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {75, 75, 105, 105},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {75, 75, 105, 105},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; 着陆时的足端位置
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {2, 2, 2, 2},
    },
    {
        // GroundLeft 原地左转跳跃

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.27, 0.27, 0.27, 0.27},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.4, 0.4, 0.4, 0.4},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {70, 70, 70, 70},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.25, 0.25, 0.25, 0.25},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0, 0, 0, 0},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {70, 70, 70, 70},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; 着陆时的足端位置
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1, 1, 1, 1},

    },
    {
        // GroundRight 原地右转跳跃

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 110, 110},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.27, 0.27, 0.27, 0.27},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.4, 0.4, 0.4, 0.4},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 110, 110},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.25, 0.25, 0.25, 0.25},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0, 0, 0, 0},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 110, 110},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; 着陆时的足端位置
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1, 1, 1, 1},
    },
    {
        // GroundUpToStep 平地到台阶

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.2, 0.2, 0.2, 0.2},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.5, 0.5, 0.5, 0.5},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.18, 0.18, 0.18, 0.18},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0.02, 0.02, 0.02, 0.02},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; 着陆时的足端位置
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1, 1, 1, 1},
    },
    {
        // StepUpToStep 上台阶到台阶

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.27, 0.27, 0.27, 0.27},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.5, 0.59, 0.5, 0.59},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; 着陆时的足端位置
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1, 1, 1, 1},
    },
    {
        // StepUpToGround 上台阶到平地

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.12, 0.12, 0.12, 0.12},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.35, 0.35, 0.35, 0.35},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.55, 0.55, 0.55, 0.55},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.34, 0.34, 0.34, 0.34},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.34, 0.34, 0.34, 0.34},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0, 0, 0, 0},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.15, 0.15, 0.15, 0.15},

        // Load_Pos[4]; 着陆时的足端位置

        .Load_Pos = {{0, 0, 0.14}, {0, 0, 0.34}, {0, 0, 0.34}, {0, 0, 0.14}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {0.5, 0.5, 0.5, 0.5},
    },
    {
        // StepDownToStep 下台阶到台阶

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.12, 0.12, 0.12, 0.12},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.35, 0.35, 0.35, 0.35},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.55, 0.55, 0.55, 0.55},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.34, 0.34, 0.34, 0.34},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.34, 0.34, 0.34, 0.34},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0, 0, 0, 0},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.15, 0.15, 0.15, 0.15},

        // Load_Pos[4]; 着陆时的足端位置
        .Load_Pos = {{0, 0, 0.14}, {0, 0, 0.34}, {0, 0, 0.34}, {0, 0, 0.14}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {0.5, 0.5, 0.5, 0.5},
    },
    {
        // StepDownToGround 下台阶到平地

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.12, 0.12, 0.12, 0.12},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.35, 0.35, 0.35, 0.35},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.55, 0.55, 0.55, 0.55},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.34, 0.34, 0.34, 0.34},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.1, 0.1, 0.1, 0.1},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.34, 0.34, 0.34, 0.34},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {110, 110, 70, 70},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0, 0, 0, 0},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.15, 0.15, 0.15, 0.15},

        // Load_Pos[4]; 着陆时的足端位置
        .Load_Pos = {{0, 0, 0.14}, {0, 0, 0.34}, {0, 0, 0.34}, {0, 0, 0.14}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {0.5, 0.5, 0.5, 0.5},
    },
    {
        // ClimbUp 上坡

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.18, 0.18, 0.18, 0.18},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {120, 120, 60, 60},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.5, 0.5, 0.5, 0.5},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.59, 0.59, 0.59, 0.59},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {120, 120, 60, 60},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.2, 0.2, 0.2, 0.2},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0.05, 0.05, 0.05, 0.05},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.18, 0.18, 0.18, 0.18},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {120, 120, 60, 60},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; 着陆时的足端位置
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1, 1, 1, 1},
    },

    {
        // ClimbDown 下坡

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {115, 115, 65, 65},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.5, 0.5, 0.5, 0.5},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {115, 115, 65, 65},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.18, 0.18, 0.18, 0.18},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0.07, 0.07, 0.07, 0.07},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.18, 0.18, 0.18, 0.18},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {115, 115, 65, 65},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.05, 0.05, 0.05, 0.05},

        // Load_Pos[4]; 着陆时的足端位置
        {{0, 0, 0.25}, {0, 0, 0.25}, {0, 0, 0.25}, {0, 0, 0.25}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1, 1, 1, 1},

    },
    {
        // JumpTest1 兔子跳

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.59, 0.59, 0.59, 0.59},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {},

        // Load_Pos[4]; 着陆时的足端位置
        {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {1, 1, 1, 1},

        .prep_angle = {30, 75, 75, 30},
        .jump_angle = {70, 80, 80, 70},
        .prep_height = {12, 6, 6, 12},

        .T.prep_time_start = {0, 0.55, 0.55, 0},    // 隔多久开始收腿
        .T.prep_time_length = {0.5, 0.1, 0.1, 0.5}, // 收腿时间长度
        .T.prep_time = {0.5, 0.5, 0.5, 0.5},        // 收腿时间 (s)

        .T.jump_time_length = {0.16, 0.1, 0.1, 0.15}, // 起跳时间长度 0.05
        .T.jump_time_damp = {0.8, 0.8, 0.8, 0.8},     // 起跳伸腿阻尼减震,暂时没用
        .T.jump_time = {0.25, 0.25, 0.25, 0.25},      // 起跳时间

        .T.tuck_time = {0.07, 0.07, 0.07, 0.07},           // 收腿时间
        .T.prep_launch_time_start = {0, 0, 0, 0},          //   开始摆腿
        .T.prep_launch_time_length = {0.1, 0.1, 0.1, 0.1}, // 摆腿时间长度
        .T.prep_launch_time = {0.3, 0.3, 0.3, 0.3},        // 准备降落的摆腿时间(算上开始时间)

    },
    {
        // JumpTest2 前腿蹦 (用于跨栏)

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {100, 100, 80, 80},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0, 0, 0, 0},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.59, 0.59, 0.59, 0.59},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {100, 100, 80, 80},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.05, 0.05, 0.05, 0.05},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0.2, 0.2, 0.2, 0.2},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {100, 100, 80, 80},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.05, 0.05, 0.05, 0.05},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.05, 0.05, 0.05, 0.05},

        // Load_Pos[4]; 着陆时的足端位置
        {{0, 0, 0.4}, {0, 0, 0.4}, {0, 0, 0.4}, {0, 0, 0.4}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {0.7, 0.7, 0.7, 0.7},

    },
    {
        // JumpTest3 后腿蹦 (用于跨栏)

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        {0.38, 0.38, 0.38, 0.38},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {102, 102, 78, 78},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        {0, 0, 0, 0},

        // Jump_Rho[4]; 起跳时关节电机到足端的距离
        {0.59, 0.59, 0.59, 0.59},

        // Jump_Theta[4]; 起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {102, 102, 78, 78},

        // Jump_Time[4]; 起跳时蹬腿动作用的时间
        {0.05, 0.05, 0.05, 0.05},

        // Jump_Buffer_Time[4]; 起跳时蹬腿完成后的缓冲时间
        {0.18, 0.18, 0.18, 0.18},

        // Tuck_Rho[4]; 起跳后在空中收腿时关节电机到足端的距离
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; 起跳后在空中收腿时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        {102, 102, 78, 78},

        // Tuck_Time[4]; 起跳后在空中收腿用的时间
        {0.05, 0.05, 0.05, 0.05},

        // Tuck_Buffer_Time[4]; 起跳后在空中收腿完成后的缓冲时间,收腿以后等待落地前摆腿
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; 收腿完成后腿的回摆时间,腿摆回去以后就落地了
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; 着陆时的足端位置
        {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; 落地缓冲时间
        {0.8, 0.8, 0.8, 0.8},
    },
    */
};
