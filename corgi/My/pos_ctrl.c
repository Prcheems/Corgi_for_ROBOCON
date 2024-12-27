#include "pos_ctrl.h"
#include "jump.h"

/**
 * @brief 足端走一条直线,位控
 * @param legid 输入腿的id,判断是哪只腿走直线
 * @param start_x,start_z 直线始端
 * @param end_x,end_z 直线末端
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @param Swing PD控制摆动相参数
 */
void line_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float T, float t)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
    {
        Get_Now_Pos_Spd(legid);
        leg[legid].Pctrl.Foot_Last_Target.p = leg[legid].Foot_Target.p;

        Point p = {0, 0, 0};
        p = line_Generator(legid, start_x, start_z, end_x, end_z, T, t);
        Coordinate_Transform(&p, &leg[legid].Foot_Target.p, Euler_Angle.Pitch, &leg[legid], legid);

        Foot_Convert_To_Motor(&leg[legid].const_param, &leg[legid].Foot_Target, &leg[legid].Pctrl.Joint_Target, &leg[legid].Pctrl.Motor_Target);
        Set_Motor_Target_Pos(&leg[legid]);
    }
}

/**
 * @brief 足端走一条摆线,位控
 * @param legid 输入腿的id,判断是哪只腿走直线
 * @param start_x,start_z 摆线始端
 * @param end_x 摆线末端
 * @param max_z 摆线顶点z坐标
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @param Swing PD控制摆动相参数
 */
void Cycloid_Pos(uint8_t legid, float start_x, float start_z, float end_x, float max_z, float T, float t)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
    {
        Get_Now_Pos_Spd(legid);
        leg[legid].Pctrl.Foot_Last_Target.p = leg[legid].Foot_Target.p;

        Point p = {0, 0, 0};
        p = Cycloid_Generator(legid, start_x, start_z, (end_x - start_x), (start_z - max_z), T, t);
        Coordinate_Transform(&p, &leg[legid].Foot_Target.p, Euler_Angle.Pitch, &leg[legid], legid);

        Foot_Convert_To_Motor(&leg[legid].const_param, &leg[legid].Foot_Target, &leg[legid].Pctrl.Joint_Target, &leg[legid].Pctrl.Motor_Target);
        Set_Motor_Target_Pos(&leg[legid]);
    }
}

/**
 * @brief 足端走贝塞尔曲线,摆动相使用,位控
 * @param legid 输入腿的id,判断是哪只腿走摆线(涉及速度取正取负)
 * @param start_x,start_z 腿坐标系下曲线始端
 * @param end_x,end_z 腿坐标系下曲线末端
 * @param max_z 曲线的最大高度,如果不限制写0,向腿坐标系z轴正向摆为-,向腿坐标系z轴负向摆为+
 * @param Delta_x,Delta_z 在腿坐标系下,贝塞尔曲线的原点坐标
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @param Swing PD控制摆动相参数
 */
void Bezier_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
    {
        Get_Now_Pos_Spd(legid);
        leg[legid].Pctrl.Foot_Last_Target.p = leg[legid].Foot_Target.p;

        Point p = {0, 0, 0};
        p = Bezier_Generator(legid, start_x, start_z, end_x, end_z, max_z, Delta_x, Delta_z, T, t);
        Coordinate_Transform(&p, &leg[legid].Foot_Target.p, Euler_Angle.Pitch, &leg[legid], legid);

        Foot_Convert_To_Motor(&leg[legid].const_param, &leg[legid].Foot_Target, &leg[legid].Pctrl.Joint_Target, &leg[legid].Pctrl.Motor_Target);
        Set_Motor_Target_Pos(&leg[legid]);
    }
}

/**
 * @brief 足端走贝塞尔曲线,walk状态支撑相使用,位控
 * @param legid 输入腿的id,判断是哪只腿走摆线(涉及速度取正取负)
 * @param start_x,start_z 腿坐标系下曲线始端
 * @param end_x,end_z 腿坐标系下曲线末端
 * @param max_z 曲线的最大高度,如果不限制写0,向腿坐标系z轴正向摆为-,向腿坐标系z轴负向摆为+
 * @param Delta_x,Delta_z 在腿坐标系下,贝塞尔曲线的原点坐标
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @param Stance 支撑相支撑的参数
 */
void Bezier_Stance_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
    {
        Get_Now_Pos_Spd(legid);
        leg[legid].Pctrl.Foot_Last_Target.p = leg[legid].Foot_Target.p;

        Point p = {0, 0, 0};
        p = Bezier_Generator(legid, start_x, start_z, end_x, end_z, max_z, Delta_x, Delta_z, T, t);
        Coordinate_Transform(&p, &leg[legid].Foot_Target.p, Euler_Angle.Pitch, &leg[legid], legid);

        Foot_Convert_To_Motor(&leg[legid].const_param, &leg[legid].Foot_Target, &leg[legid].Pctrl.Joint_Target, &leg[legid].Pctrl.Motor_Target);
        Set_Motor_Target_Pos(&leg[legid]);
    }
}

// 位控
void gait_posctrl(uint8_t legid, GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
    {
        if (gait_param->Leg_State[legid] == Swing)
        {
            switch (legid) // 腿2腿3在x方向上取反
            {
            case 0:
                Bezier_Pos(legid,
                           Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                           gait_param->step_length[0] / 2.0f, gait_param->stance_height,
                           gait_param->step_height,
                           0, gait_param->stance_height,
                           0.5f * gait_param->term * gait_param->swing_percent, t);
                break;
            case 1:
                Bezier_Pos(legid,
                           Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                           gait_param->step_length[1] / 2.0f, gait_param->stance_height,
                           gait_param->step_height,
                           0, gait_param->stance_height,
                           0.5f * gait_param->term * gait_param->swing_percent, t - 0.5f * gait_param->term);
                break;
            case 2:
                Bezier_Pos(legid,
                           Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                           -gait_param->step_length[2] / 2.0f, gait_param->stance_height,
                           gait_param->step_height,
                           0, gait_param->stance_height,
                           0.5f * gait_param->term * gait_param->swing_percent, t); // 这里腿2腿3的初始位置必须取反！！！！！
                break;
            case 3:
                Bezier_Pos(legid,
                           Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                           -gait_param->step_length[3] / 2.0f, gait_param->stance_height,
                           gait_param->step_height,
                           0, gait_param->stance_height,
                           0.5f * gait_param->term * gait_param->swing_percent, t - 0.5f * gait_param->term); // 这里腿2腿3的初始位置必须取反！！！！！
                break;
            }
        }

        else if (gait_param->Leg_State[legid] == Stance)
        {
            switch (legid)
            {
            case 0:
                Bezier_Stance_Pos(legid,
                                  Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                                  -gait_param->step_length[0] / 2.0f, gait_param->stance_height,
                                  -gait_param->step_height_down,
                                  0, gait_param->stance_height,
                                  0.5f * gait_param->term * gait_param->swing_percent, t - 0.5f * gait_param->term);
                break;
            case 1:
                Bezier_Stance_Pos(legid,
                                  Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                                  -gait_param->step_length[1] / 2.0f, gait_param->stance_height,
                                  -gait_param->step_height_down,
                                  0, gait_param->stance_height,
                                  0.5f * gait_param->term * gait_param->swing_percent, t);
                break;
            case 2:
                Bezier_Stance_Pos(legid,
                                  Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                                  gait_param->step_length[2] / 2.0f, gait_param->stance_height,
                                  -gait_param->step_height_down,
                                  0, gait_param->stance_height,
                                  0.5f * gait_param->term * gait_param->swing_percent, t - 0.5f * gait_param->term);
                break;
            case 3:
                Bezier_Stance_Pos(legid,
                                  Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                                  gait_param->step_length[3] / 2.0f, gait_param->stance_height,
                                  -gait_param->step_height_down,
                                  0, gait_param->stance_height,
                                  0.5f * gait_param->term * gait_param->swing_percent, t);
                break;
            }
        }
    }
}

void Walk_Away_Posctrl(GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t)
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
    if ((((gait_param->t) >= 0) && ((gait_param->t) < (0.5f * gait_param->term)) && ((gait_param->last_t) > (0.5f * gait_param->term))) ||
        (((gait_param->t) <= 0.001f) && ((gait_param->last_t <= 0.001f)))) // 腿0腿2摆动相,腿1腿3支撑相
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
    }
    gait_posctrl(0, gait_param, Action_Param, Euler_Angle, gait_param->t);
    gait_posctrl(1, gait_param, Action_Param, Euler_Angle, gait_param->t);
    gait_posctrl(2, gait_param, Action_Param, Euler_Angle, gait_param->t);
    gait_posctrl(3, gait_param, Action_Param, Euler_Angle, gait_param->t);

    if (gait_param->Walk_State == Stop)
    {
        SetDogChangeAble(ENABLE);
        Action_Param->Launch_Flag = Finish;
    }
}

Pos_motor prep_param = {
    .kp = 0.2,
    .kw = 0.1,
};
Pos_motor jump_param = {
    .kp = 0.5,
    .kw = 0.1,
};
Pos_motor luanch_param = {
    .kp = 0.002,
    .kw = 5,
};

bool Jump_s_pos(uint8_t legid, JumpParam *Jump_Param, action_param *Action_Param, Euler *Euler_Angle, float t)
{
    bool Jump_State = false;
    if (legid <= 3 && legid >= 0) // 严防数组越界
    {
        // 时间安排
        float t0 = 0;
        float t1 = t0 + Jump_Param->Prep_Time[legid] + Jump_Param->Prep_Buffer_Time[legid];
        float t2 = t1 + Jump_Param->Jump_Time[legid];
        float t3 = t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid] + Jump_Param->Swing_in_Air_Time[legid];
        float t4 = t3 + Jump_Param->Land_Buffer_Time[legid];

        // 跳跃过程Start!
        if (t <= t0) // 什么都不做
        {
        }
        else if ((t > t0) && (t <= t1)) // 收腿准备起跳
        {
            Set_All_Motor_Param(&prep_param);
            line_Pos(legid,
                     Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                     Jump_Param->Prep_Rho[legid] * cosf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * sinf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Time[legid], t - t0);
        }
        else if ((t > t1) && (t <= t2)) // 起跳
        {
            if ((t > t1) && (t <= t1 + Jump_Param->Jump_Time[legid]))
            {
                Set_All_Motor_Param(&jump_param);
                line_Pos(legid,
                         Jump_Param->Prep_Rho[legid] * cosf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Prep_Rho[legid] * sinf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Jump_Time[legid], t - t1);
            }
            else if ((t > t1 + Jump_Param->Jump_Time[legid]) && (t <= t1 + Jump_Param->Jump_Time[legid] + Jump_Param->Jump_Buffer_Time[legid]))
            {
                line_Pos(legid,
                         Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Jump_Buffer_Time[legid], t - t1 - Jump_Param->Jump_Time[legid]);
            }
        }
        else if ((t > t2) && (t <= t3)) // 空中摆腿
        {
            if (t <= (t2 + Jump_Param->Tuck_Time[legid])) // 空中收腿
            {
                line_Pos(legid,
                         Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Time[legid], t - t2);
            }
            else if ((t > (t2 + Jump_Param->Tuck_Time[legid])) && (t <= (t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid]))) // 什么都不干
            {
                line_Pos(legid,
                         Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Buffer_Time[legid], t - t2 - Jump_Param->Tuck_Time[legid]);
            }

            else if (t > (t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid])) // 准备着陆
            {
                line_Pos(legid,
                         Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Load_Rho[legid] * cosf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Load_Rho[legid] * sinf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Swing_in_Air_Time[legid],
                         t - t2 - Jump_Param->Tuck_Time[legid] - Jump_Param->Tuck_Buffer_Time[legid]);
            }
        }
        else if (t > t3 && t <= t4) // 阻尼减震
        {
            Set_All_Motor_Param(&luanch_param);
            Keep_One_Leg(legid,
                         Jump_Param->Load_Rho[legid] * cosf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Load_Rho[legid] * sinf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN), Jump_VMC.VMC_Cushion);
        }

        else if (t > t4)
        {
            Jump_State = true;
        }
    }
    return Jump_State;
}

void Change_Ctrl_State(Ctrl_State state)
{
    leg[0].ctrl_state = state;
}

Ctrl_State Get_Now_Ctrl_State(void)
{
    return leg[0].ctrl_state;
}
