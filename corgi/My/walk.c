#include "dog.h"
#include "walk.h"
#include "action.h"
#include "math.h"
#include "timer.h"
#include "delay_DWT.h"
#include "pos_ctrl.h"
#include "imu.h"
#include "A1_motor.h"
#include "pos_ctrl.h"

#define STAND_HEIGHT 0.2f
#define STEP_LENGTH 0.15f // 步长最多0.15m
#define STEP_HEIGHT 0.05f

GaitParam gait_param = {
    .Leg_State = {Swing, Stance, Swing, Stance}, // 腿的状态
    .Walk_State = First,                         // walk状态
    .Dire = Front,
    .stance_height = STAND_HEIGHT,                                       // 站立高度(m)
    .step_length = {STEP_LENGTH, STEP_LENGTH, STEP_LENGTH, STEP_LENGTH}, // 步长(m)
    .step_height = STEP_HEIGHT,                                          // 步高(m)
                                                                         //    .step_height_down = 0.05,                                            // 支撑腿下沉步高
    .step_height_down = 0,                                               // 支撑腿下沉步高
    .std_step_length = STEP_LENGTH,                                      // 标准步长,方便赋值(m)

    .swing_percent = 0.99f, // 摆动相百分比
    .term = 0.2f,          // 步态周期(s)
    .t = 0,                // 当前时间
    .last_t = 0,           // 暂存上次的时间,用于支撑摆动腿切换

    .VMC_Swing_Walk = {
        {1800, 2500, 0.5, 3500, 4000, 0.5, 0, 0},
        {1800, 2500, 0.5, 3500, 4000, 0.5, 0, 0},
        {1800, 900,  0.5, 3500, 4000, 0.5, 0, 0},
        {1800, 2500, 0.5, 3500, 4000, 0.5, 0, 0},
    },

    .VMC_Stance_Walk = {
        {3300, 300, 0.5, 6000, 2000, 0.5, 0, 60},
        {3300, 300, 0.5, 6000, 2000, 0.5, 0, 60},
        {3300, 300, 0.5, 6000, 2000, 0.5, 0, 60},
        {3300, 844, 0.5, 6000, 2000, 0.5, 0, 60},
    },

    .VMC_Euler_Walk = {0.0035, 0.000005, 0.5}, //
    .Yaw_max = 0.05,
    .Posctrl_Stance = {0, 0.04, 1.5},
    .Posctrl_Swing = {0, 0.04, 1.5},
    .Scope = {0, 0.2},
};

extern float data2send[10];

/**
 * @brief 把陀螺仪的反馈Pitch叠加到步高和步态x方向对称轴上提高走坡时稳定性
 * @param Euler 欧拉角结构体,用于读取当前偏航角和目标偏航角
 * @param gait_param 步态结构体,步态x方向对称轴位置
 * @warning 由于这里直接在step_length基础上-的,如果step_length不基于std_step_length一直更新数据(这里是用Yaw_Pos_Crol)
 *          实现更新的,那么步长会一直减小到离谱的程度,会损坏结构！！！！
 */
void Pitch_Pos_Offest(Euler *Euler, GaitParam *gait_param)
{
//    if (Euler->Pitch >= 5.0f || Euler->Pitch <= -5.0f)
//    {
//        gait_param->stance_height = STAND_HEIGHT - ((fabs(Euler->Pitch) - 5.0f) / 10.0f) * (STAND_HEIGHT - gait_param->Scope.Std_Height);
//        gait_param->Scope.Axis_Offset = gait_param->stance_height * tanf(Euler->Pitch * 0.017453) * 0.8f;

//        if (gait_param->Dire == Back)
//        {
//            gait_param->Scope.Axis_Offset = -gait_param->Scope.Axis_Offset;
//        }

//        if (gait_param->Leg_State[0] == Swing)
//            gait_param->step_length[0] = gait_param->step_length[0] + gait_param->Scope.Axis_Offset;
//        else if (gait_param->Leg_State[0] == Stance)
//            gait_param->step_length[0] = gait_param->step_length[0] - gait_param->Scope.Axis_Offset;

//        if (gait_param->Leg_State[1] == Swing)
//            gait_param->step_length[1] = gait_param->step_length[1] + gait_param->Scope.Axis_Offset;
//        else if (gait_param->Leg_State[1] == Stance)
//            gait_param->step_length[1] = gait_param->step_length[1] - gait_param->Scope.Axis_Offset;

//        if (gait_param->Leg_State[2] == Swing)
//            gait_param->step_length[2] = gait_param->step_length[2] + gait_param->Scope.Axis_Offset;
//        else if (gait_param->Leg_State[2] == Stance)
//            gait_param->step_length[2] = gait_param->step_length[2] - gait_param->Scope.Axis_Offset;

//        if (gait_param->Leg_State[3] == Swing)
//            gait_param->step_length[3] = gait_param->step_length[3] + gait_param->Scope.Axis_Offset;
//        else if (gait_param->Leg_State[3] == Stance)
//            gait_param->step_length[3] = gait_param->step_length[3] - gait_param->Scope.Axis_Offset;
//    }
//    else
//    {
//        gait_param->stance_height = STAND_HEIGHT;
//    }
}

/**
 * @brief 把陀螺仪的反馈Yaw叠加到步长上矫正偏航角
 * @param Euler 欧拉角结构体,用于读取当前偏航角和目标偏航角
 * @param gait_param 步态结构体,修改步长
 * @param Upper_Limit 在步长基准std_step_length以外最大可增大步长
 * @param Lower_Limit 在步长基准std_step_length以外最大可减小步长
 */
void Yaw_Pos_Crol(Euler *Euler, GaitParam *gait_param, float Max_Changable_Step_Length)
{
    // 计算误差error
    gait_param->VMC_Euler_Walk.err_x = Euler->Target_Yaw - Euler->Yaw;

    // 计算不完全D项
    gait_param->VMC_Euler_Walk.differ_x = gait_param->VMC_Euler_Walk.Kd_x * (1 - gait_param->VMC_Euler_Walk.k_x) *
                                              (gait_param->VMC_Euler_Walk.err_x - gait_param->VMC_Euler_Walk.last_err_x) +
                                          gait_param->VMC_Euler_Walk.k_x * gait_param->VMC_Euler_Walk.last_differ_x;

    // 计算输出
    float Delta_p = gait_param->VMC_Euler_Walk.Kp_x * gait_param->VMC_Euler_Walk.err_x + gait_param->VMC_Euler_Walk.differ_x;

    // 传递参数
    gait_param->VMC_Euler_Walk.last_err_x = gait_param->VMC_Euler_Walk.err_x;
    gait_param->VMC_Euler_Walk.last_differ_x = gait_param->VMC_Euler_Walk.differ_x;

    if (gait_param->Dire == Front)
    {
        gait_param->step_length[0] = gait_param->std_step_length - Delta_p;
        gait_param->step_length[1] = gait_param->std_step_length - Delta_p;
        gait_param->step_length[2] = gait_param->std_step_length + Delta_p;
        gait_param->step_length[3] = gait_param->std_step_length + Delta_p;

        for (int i = 0; i <= 3; i++)
        {
            if (gait_param->step_length[i] >= (Max_Changable_Step_Length + gait_param->std_step_length))
            {
                gait_param->step_length[i] = Max_Changable_Step_Length + gait_param->std_step_length;
            }
            else if (gait_param->step_length[i] <= 0)
            {
                gait_param->step_length[i] = 0;
            }
        }
    }
    if (gait_param->Dire == Back)
    {
        gait_param->step_length[0] = -gait_param->std_step_length - Delta_p;
        gait_param->step_length[1] = -gait_param->std_step_length - Delta_p;
        gait_param->step_length[2] = -gait_param->std_step_length + Delta_p;
        gait_param->step_length[3] = -gait_param->std_step_length + Delta_p;

        for (int i = 0; i <= 3; i++)
        {
            if (gait_param->step_length[i] <= (-Max_Changable_Step_Length - gait_param->std_step_length))
            {
                gait_param->step_length[i] = -Max_Changable_Step_Length - gait_param->std_step_length;
            }
            else if (gait_param->step_length[i] >= 0)
            {
                gait_param->step_length[i] = 0;
            }
        }
    }
}

/*
void Yaw_Pos_Crol(Euler *Euler, GaitParam *gait_param, float Upper_Limit, float Lower_Limit)
{
        float Delta_p = gait_param->VMC_Euler_Walk.Kp_yaw * (Euler->Target_Yaw - Euler->Yaw) - gait_param->VMC_Euler_Walk.Kw_yaw * (Euler->Last_Yaw - Euler->Yaw);
        if (Delta_p >= Max_Changable_Step_Length)
    {
        Delta_p = Max_Changable_Step_Length;
    }
    else if (Delta_p <= -Max_Changable_Step_Length)
    {
        Delta_p = -Max_Changable_Step_Length;
    }

    if (gait_param->Dire == Front)
    {
        gait_param->step_length[0] = gait_param->std_step_length - Delta_p;
        gait_param->step_length[1] = gait_param->std_step_length - Delta_p;
        gait_param->step_length[2] = gait_param->std_step_length + Delta_p;
        gait_param->step_length[3] = gait_param->std_step_length + Delta_p;
    }
    if (gait_param->Dire == Back)
    {
        gait_param->step_length[0] = -gait_param->std_step_length - Delta_p;
        gait_param->step_length[1] = -gait_param->std_step_length - Delta_p;
        gait_param->step_length[2] = -gait_param->std_step_length + Delta_p;
        gait_param->step_length[3] = -gait_param->std_step_length + Delta_p;
    }
}
*/

void gait(uint8_t legid, GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
    {
        if (gait_param->Leg_State[legid] == Swing)
        {
            switch (legid) // 腿2腿3在x方向上取反
            {
            case 0:
                Bezier(legid,
                       Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                       gait_param->step_length[0] / 2.0f, gait_param->stance_height,
                       gait_param->step_height,
                       0, gait_param->stance_height,
                       0.5f * gait_param->term * gait_param->swing_percent, t, &gait_param->VMC_Swing_Walk[0]);
                break;
            case 1:
                Bezier(legid,
                       Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                       gait_param->step_length[1] / 2.0f, gait_param->stance_height,
                       gait_param->step_height,
                       0, gait_param->stance_height,
                       0.5f * gait_param->term * gait_param->swing_percent, t - 0.5f * gait_param->term, &gait_param->VMC_Swing_Walk[1]);
                break;
            case 2:
                Bezier(legid,
                       Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                       -gait_param->step_length[2] / 2.0f, gait_param->stance_height,
                       gait_param->step_height,
                       0, gait_param->stance_height,
                       0.5f * gait_param->term * gait_param->swing_percent, t, &gait_param->VMC_Swing_Walk[2]); // 这里腿2腿3的初始位置必须取反！！！！！
                break;
            case 3:
                Bezier(legid,
                       Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                       -gait_param->step_length[3] / 2.0f, gait_param->stance_height,
                       gait_param->step_height,
                       0, gait_param->stance_height,
                       0.5f * gait_param->term * gait_param->swing_percent, t - 0.5f * gait_param->term, &gait_param->VMC_Swing_Walk[3]); // 这里腿2腿3的初始位置必须取反！！！！！
                break;
            }
        }

        else if (gait_param->Leg_State[legid] == Stance)
        {
            switch (legid)
            {
            case 0:
                Bezier_Stance(legid,
                              Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                              -gait_param->step_length[0] / 2.0f, gait_param->stance_height,
                              -gait_param->step_height_down,
                              0, gait_param->stance_height,
                              0.5f * gait_param->term * gait_param->swing_percent, t - 0.5f * gait_param->term, &gait_param->VMC_Stance_Walk[0]);
                break;
            case 1:
                Bezier_Stance(legid,
                              Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                              -gait_param->step_length[1] / 2.0f, gait_param->stance_height,
                              -gait_param->step_height_down,
                              0, gait_param->stance_height,
                              0.5f * gait_param->term * gait_param->swing_percent, t, &gait_param->VMC_Stance_Walk[1]);
                break;
            case 2:
                Bezier_Stance(legid,
                              Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                              gait_param->step_length[2] / 2.0f, gait_param->stance_height,
                              -gait_param->step_height_down,
                              0, gait_param->stance_height,
                              0.5f * gait_param->term * gait_param->swing_percent, t - 0.5f * gait_param->term, &gait_param->VMC_Stance_Walk[2]);
                break;
            case 3:
                Bezier_Stance(legid,
                              Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                              gait_param->step_length[3] / 2.0f, gait_param->stance_height,
                              -gait_param->step_height_down,
                              0, gait_param->stance_height,
                              0.5f * gait_param->term * gait_param->swing_percent, t, &gait_param->VMC_Stance_Walk[3]);
                break;
            }
        }
    }
}

// 目标位置采用当前位置累加一个步长还是初始位置加n个步长
// 如果有某一条腿偏离预定位置过多怎么办
// 写一个复位回原始keep状态的函数
// 支撑相的腿应该始终位于什么地方,启动时的单独位移差?
void Walk_Away(GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t)
{
    // 因为步态是周期运动,把时间t缩短到一个周期中
    if (t >= gait_param->term)
    {
        float t_former = t;             // 暂存原来的时间t
        t = fmodf(t, gait_param->term); // 对t按term取余,得到0~term的一个数
        // gait_param->Cycle_Index = round((t_former - t) / gait_param->term); // 求出从开始到t时刻一共经历了几个周期,后面用于给定足端目标位置
    }
    gait_param->last_t = gait_param->t; // t存储的是当前时间,而gait_param->last_t和gait_param->t是为了记录上一次进入walk_away函数时候的时间
    gait_param->t = t;

    SetDogChangeAble(DISABLE);
    // 腿的状态切换 时序
    // 如果当前时间gait_param->t在0`0.5周期,且上一刻的时间大于0.5周期(即上一刻是一个步态周期的结尾),切换一次状态
    if ((((gait_param->t) >= 0) && ((gait_param->t) < (0.5f * gait_param->term)) &&
         ((gait_param->last_t) > (0.5f * gait_param->term)) && ((gait_param->last_t) <= (gait_param->term))) ||
        (((gait_param->t) <= 0.001) && ((gait_param->last_t <= 0.001)))) // 腿0腿2摆动相,腿1腿3支撑相
    {
        // 切换状态
        gait_param->Leg_State[0] = Swing;
        gait_param->Leg_State[2] = Swing;
        gait_param->Leg_State[1] = Stance;
        gait_param->Leg_State[3] = Stance;

        SetDogChangeAble(ENABLE);

        // 切换状态的这一刻获取步态起始点
        for (int i = 0; i <= 3; i++)
        {
            Get_Now_Pos_Spd(i);
            Euler_Update(Euler_Angle);
            Coordinate_Transform(&leg[i].Foot.p, &Action_Param->Init_Pos[i], -Euler_Angle->Pitch, &leg[i], i);
        }
        /*
                if (gait_param->Walk_State == First)
                {
                    gait_param->step_length = gait_param->std_step_length / 2.0f;
                    gait_param->Walk_State = Later;
                }
        */
        // 偏航角矫正
        Yaw_Pos_Crol(Euler_Angle, gait_param, gait_param->Yaw_max);

        // 俯仰角用于修改步态
        Pitch_Pos_Offest(Euler_Angle, gait_param);
    }
    // 如果当前时间gait_param->t在0.5`1个周期,且上一刻的时间小于0.5周期(即上一刻是半个步态周期的结尾),切换一次状态
    else if ((((gait_param->t) >= (0.5f * gait_param->term) && ((gait_param->t) < (gait_param->term))) &&
              (((gait_param->last_t) > 0) && (gait_param->last_t) < (0.5f * gait_param->term))) ||
             ((gait_param->t) >= (0.5f * gait_param->term) && (gait_param->t) <= (0.5f * gait_param->term + 0.001) &&
              ((gait_param->last_t) >= (0.5f * gait_param->term)) && (gait_param->last_t) <= (0.5f * gait_param->term + 0.001))) // 腿1腿3摆动相,腿0腿2支撑相
    {
        // 切换状态
        gait_param->Leg_State[1] = Swing;
        gait_param->Leg_State[3] = Swing;
        gait_param->Leg_State[0] = Stance;
        gait_param->Leg_State[2] = Stance;

        SetDogChangeAble(ENABLE);

        // 切换状态的这一刻获取步态起始点
        for (int i = 0; i <= 3; i++)
        {
            Get_Now_Pos_Spd(i);
            Euler_Update(Euler_Angle);
            Coordinate_Transform(&leg[i].Foot.p, &Action_Param->Init_Pos[i], -Euler_Angle->Pitch, &leg[i], i);
        }

        // 偏航角矫正
        Yaw_Pos_Crol(Euler_Angle, gait_param, gait_param->Yaw_max);

        // 俯仰角用于修改步态
        Pitch_Pos_Offest(Euler_Angle, gait_param);

        /*if (gait_param->Walk_State == Later)
        {
            gait_param->step_length = gait_param->std_step_length;
        }*/
    }
    /*if (gait_param->Walk_State == Later)
    {*/
    gait(0, gait_param, Action_Param, Euler_Angle, gait_param->t);
    gait(1, gait_param, Action_Param, Euler_Angle, gait_param->t);
    gait(2, gait_param, Action_Param, Euler_Angle, gait_param->t);
    gait(3, gait_param, Action_Param, Euler_Angle, gait_param->t);
    /*}
    else */
    if (gait_param->Walk_State == Stop)
    {
        SetDogChangeAble(ENABLE);
        Action_Param->Launch_Flag = Finish;
    }
}
// 外部函数修改Walk参数的接口
void Set_Target_Yaw(float Target_Yaw)
{
    Euler_Angle.Target_Yaw = Target_Yaw;
}

void Set_Walk_Dire(dire_state Dire)
{
    gait_param.Dire = Dire;
}

void Set_Walk_Length(float Walk_Length)
{
    gait_param.std_step_length = Walk_Length;
}

void Set_Turning_Kp(float Kp)
{
    gait_param.VMC_Euler_Walk.Kp_x = Kp;
}

static float tick_start = 0; // 动作的开始时间(s)

void walk_enter()
{
    gait_param.t = 0;
    SetDogChangeAble(DISABLE);     // 动作完成前禁止切换状态
    gait_param.Walk_State = First; // 设置walk状态为第一次迈步
    Action_Param.Launch_Flag = Launch;
    GetTick();
    tick_start = Time_Points_Param.Now_Tick; // 获取当前时间
//    Yaw_Reset(&Euler_Angle);
    // Change_Dog_Pitch_State(Follow_Ground);
    for (int i = 0; i <= 3; i++)
    {
        Get_Now_Pos_Spd(i);
        Euler_Update(&Euler_Angle);
        Coordinate_Transform(&leg[i].Foot.p, &Action_Param.Init_Pos[i], -Euler_Angle.Pitch, &leg[i], i);
    }
    Set_All_Motor_Param(&gait_param.Posctrl_Stance);
}

void walk_run()
{
    SetDogChangeAble(DISABLE);
    GetTick();
    float t = Time_Points_Param.Now_Tick - tick_start;

    if (Get_Now_Ctrl_State() == Force_Ctrl)
    {
        Walk_Away(&gait_param, &Action_Param, &Euler_Angle, t); // 力控
    }
    if (Get_Now_Ctrl_State() == Pos_Ctrl)
    {
        Walk_Away_Posctrl(&gait_param, &Action_Param, &Euler_Angle, t); // 位控
    }
}

void walk_exit()
{
}
