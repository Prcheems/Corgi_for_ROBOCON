#include "jump.h"
#include "A1_motor.h"
#include "timer.h"
#include "cmsis_os.h"
#include "walk.h"
#include "timer.h"
#include "delay_DWT.h"
#include "Backflip.h"
#include "dog.h"
#include "pos_ctrl.h"
#include "action.h"

static float tick_start = 0; // 动作的开始时间(s)

Backflip_VMC_Param Backflip_VMC =
    {
        .VMC_Cushion = {{2000, 3500, 0.5, 2000, 5000, 0.5, 0, 0},
                        {2000, 3500, 0.5, 2000, 5000, 0.5, 0, 0},
                        {2000, 3500, 0.5, 2000, 5000, 0.5, 0, 0},
                        {2000, 3500, 0.5, 2000, 5000, 0.5, 0, 0}},

        .VMC_Swing = {{5000, 500, 0.5, 9000, 500, 0.5, 0, 0},
                      {5000, 500, 0.5, 9000, 500, 0.5, 0, 0},
                      {5000, 500, 0.5, 9000, 500, 0.5, 0, 0},
                      {5000, 500, 0.5, 9000, 500, 0.5, 0, 0}},
};

float wait_for_jump = 0;        // 等待俯仰角符合条件时记录的中间时刻
uint8_t wait_for_jump_flag = 0; // 标志位,0表示尚未记录时刻,1表示已经记录时刻,但俯仰角尚未符合条件,2表示可以进入第二次跳跃
uint8_t memory_flag;

/* ---------------------------------------后空翻过程函数----------------------------------------- */

/*
t0  +  Prep_Time  +  Prep_Buffer_Time  +  Backflip_1_Time  +  Shift_Time  +  Backflip_2_Time  +  Swing_in_Air_Time  +  Land_Buffer_Time
     准备过程中收腿    收腿以后缓冲          后腿起跳时间   后腿摆到和身体平行    前腿起跳时间     落地前腿摆到适当位置     落地缓冲时间
--------------t1---------------------
---------------------------------t2-------------------------
-----------------------------------------------------------t3-------------------------------------------------------
------------------------------------------------------------------------t4----------------------------------------------------------------
------------------------------------------------------------------------------------t5----------------------------------------------------------------------------------------

*/
bool Backflip_s(uint8_t legid, BackflipParam *Param, action_param *Action_Param, Euler *Euler_Angle, float t)
{
    bool Jump_State = false;
    if (legid <= 3 && legid >= 0) // 防数组越界
    {
        // 后空翻开始,keep状态节点
        float t0 = 0;

        // 下蹲蓄力过程,蓄力完节点
        float t1 = t0 + Param->Prep_Time[legid] + Param->Prep_Buffer_Time[legid];

        // 后腿(1,2)第一次起跳,直到身体转过75度左右节点
        float t2 = t1 + Param->Backflip_1_Time[legid];

        // 后腿(1,2)摆到与身体成一直线(90度左右),前腿摆过30度左右,进行第二次起跳(前腿0,3),再有一个缓冲节点
        float t3 = t2 + Param->Shift_Time[legid] + Param->Backflip_2_Time[legid];

        // 四条腿摆到预备落地的位置,准备落地并落地节点
        float t4 = t3 + Param->Swing_in_Air_Time[legid];

        // 落地缓冲,动作结束
        float t5 = t4 + Param->Land_Buffer_Time[legid];

        if (t <= t0) // 什么都不做
        {
            // 0 ~ t0
            // line函数在动作未开始时在原地停着因此运动始末点均为同一点,即足端初始位置点
            // 停留时间t0,采用VMC_Swing参数
            line(legid,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 t0, t - t0, &Backflip_VMC.VMC_Swing[legid]);
        }
        else if ((t > t0) && (t < t1)) // 四条腿下蹲,收腿起跳
        {
            if (t <= (t0 + Param->Prep_Time[legid]))
            {
                // t0 ~ (t0+Prep_Time)
                // line函数从Init_Pos到收腿目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
                // 运动时间Prep_Time,采用VMC_Swing参数
                line(legid,
                     Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Time[legid], t - t0, &Backflip_VMC.VMC_Swing[legid]);
            }
            else if (t > (t0 + Param->Prep_Time[legid]))
            {
                // 保持不动
                // line函数在收腿目标位置稍作停留,使身体稳定,因此运动始末点均为同一点,即预设起跳点
                // 停留时间Prep_Buffer_Time,采用VMC_Swing参数
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Buffer_Time[legid], t - t0 - Param->Prep_Time[legid], &Backflip_VMC.VMC_Swing[legid]);
            }
        }
        else if ((t > t1) && (t <= t2)) // 起跳
        {
            if (legid == 1 || legid == 2)
            {
                Change_Single_Pitch_State(Follow_Dog, 1);
                Change_Single_Pitch_State(Follow_Dog, 2);

                // 走直线,后腿起跳！
                // line函数从收腿位置到蹬腿起跳目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
                // 运动时间Backflip_1_Time,采用VMC_Swing参数
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Rho[legid] * cosf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Rho[legid] * sinf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Time[legid], t - t1, &Backflip_VMC.VMC_Swing[legid]);
            }
            if (legid == 0 || legid == 3)
            {
                // 前腿0和3随身体摆动自然摆动,保证腿和地面垂直
                Change_Single_Pitch_State(Follow_Ground, 0);
                Change_Single_Pitch_State(Follow_Ground, 3);
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     1, 1, &Backflip_VMC.VMC_Swing[legid]);
            }
        }
        else if ((t > t2) && (t <= t3)) // 腿摆到位置准备进行第二次前腿(0,3)的起跳
        {
            if (legid == 1 || legid == 2)
            {
                if (t <= (t2 + Param->Shift_Time[legid])) // 腿摆动到预备第二次起跳的位置
                {
                    // 后腿1和2摆到与身体成一直线
                    // 摆动时间Shift_Time
                    Change_Single_Pitch_State(Follow_Dog, 1);
                    Change_Single_Pitch_State(Follow_Dog, 2);
                    Bezier(legid,
                           Param->Backflip_1_Rho[legid] * cosf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                           Param->Backflip_1_Rho[legid] * sinf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                           Param->Shift_Rho[legid] * cosf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                           Param->Shift_Rho[legid] * sinf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                           -0.1f,
                           (Param->Backflip_1_Rho[legid] * cosf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN) + Param->Shift_Rho[legid] * cosf(Param->Shift_Theta[legid] * REG_TO_RADIN)) / 2.0f,
                           (Param->Backflip_1_Rho[legid] * sinf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN) + Param->Shift_Rho[legid] * sinf(Param->Shift_Theta[legid] * REG_TO_RADIN)) / 2.0f,
                           Param->Shift_Time[legid], t - t2, &Backflip_VMC.VMC_Swing[legid]);
                }
            }
            if (legid == 0 || legid == 3)
            {
                // 前腿0和3随身体摆动自然摆动,保证腿和地面垂直
                Change_Single_Pitch_State(Follow_Ground, 0);
                Change_Single_Pitch_State(Follow_Ground, 3);
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Shift_Time[legid], t - t2, &Backflip_VMC.VMC_Swing[legid]);
            }
        }

        // 俯仰角条件：当身体向前转过90度再跳,不然别跳(因为是往前转,所以拿Pitch跟-90度进行比较)
        if ((t > (t2 + Param->Shift_Time[legid])) && (fmodf(fabs(Euler_Angle->Pitch), 360.0f) <= 90.0f))
        {
            if (wait_for_jump_flag == 0) // 第一次进入判断的时候刚刚起跳,俯仰角肯定不符合条件,且wait_for_jump_flag为0,此时记录一下时间,存到wait_for_jump里面
            {
                wait_for_jump = t;
                wait_for_jump_flag = 1;
            }
            Change_Single_Pitch_State(Follow_Ground, 0);
            Change_Single_Pitch_State(Follow_Ground, 3);

            // 什么都不做,保持位置
            if (legid == 1 || legid == 2)
            {
                line(legid,
                     Param->Shift_Rho[legid] * cosf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                     Param->Shift_Rho[legid] * sinf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                     Param->Shift_Rho[legid] * cosf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                     Param->Shift_Rho[legid] * sinf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                     wait_for_jump, wait_for_jump, &Backflip_VMC.VMC_Swing[legid]);
            }
            if (legid == 0 || legid == 3)
            {
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     wait_for_jump, wait_for_jump, &Backflip_VMC.VMC_Swing[legid]);
            }
            t = wait_for_jump; // 之后t = wait_for_jump肯定在(t2+Shift_Time)~t3之间,只要下面t2~t3中加一个标志位判断即可
        }
        else if ((t > (t2 + Param->Shift_Time[legid])) && (fmodf(fabs(Euler_Angle->Pitch), 360.0f) > 90.0f))
        {
            if (wait_for_jump_flag == 1)
            {
                wait_for_jump = (t - wait_for_jump); // 现在wait_for_jump就是等待的delta_t
                wait_for_jump_flag = 2;
            }
            t = t - wait_for_jump;
        }

        if ((t > (t2 + Param->Shift_Time[legid])) && (t <= t3) && (wait_for_jump_flag == 2)) // 腿摆到位置,并且俯仰角符合条件,进行第二次前腿(0,3)的起跳
        {
            Change_Single_Pitch_State(Follow_Dog, 0);
            Change_Single_Pitch_State(Follow_Dog, 3);

            if (memory_flag == 0)
            {
                for (int i = 0; i <= 3; i++)
                {
                    Get_Now_Pos_Spd(i);
                    Action_Param->Init_Pos[i].x = leg[i].Foot.p.x;
                    Action_Param->Init_Pos[i].z = leg[i].Foot.p.z;
                }
                memory_flag = 1;
            }

            // 第二次前腿起跳
            // 走直线
            // line函数从收腿位置到蹬腿起跳目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
            // 运动时间Backflip_2_Time,采用VMC_Swing参数
            line(legid,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 Param->Backflip_2_Rho[legid] * cosf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                 Param->Backflip_2_Rho[legid] * sinf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                 Param->Backflip_2_Time[legid], t - (t2 + Param->Shift_Time[legid]), &Backflip_VMC.VMC_Swing[legid]);
        }
        else if ((t > t3) && (t <= t4) && (wait_for_jump_flag == 2)) // 四条腿摆到预备落地的位置,落地
        {                                                            // t3 ~ t4
            // Bezier函数从第二次跳完后的腿的位置到落地时预设目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
            // 运动时间Swing_in_Air_Time,采用VMC_Swing参数
            Bezier(legid,
                   Param->Backflip_2_Rho[legid] * cosf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                   Param->Backflip_2_Rho[legid] * sinf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                   Param->Load_Rho[legid] * cosf(Param->Load_Theta[legid] * REG_TO_RADIN),
                   Param->Load_Rho[legid] * sinf(Param->Load_Theta[legid] * REG_TO_RADIN),
                   -0.1f,
                   (Param->Backflip_2_Rho[legid] * cosf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN) + Param->Load_Rho[legid] * cosf(Param->Load_Theta[legid] * REG_TO_RADIN)) / 2.0f,
                   (Param->Backflip_2_Rho[legid] * sinf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN) + Param->Load_Rho[legid] * sinf(Param->Load_Theta[legid] * REG_TO_RADIN)) / 2.0f,
                   Param->Swing_in_Air_Time[legid],
                   t - t3, &Backflip_VMC.VMC_Swing[legid]);
        }

        else if ((t > t4) && (t <= t5) && (wait_for_jump_flag == 2)) // 阻尼减震
        {
            Keep_One_Leg(legid,
                         Param->Load_Rho[legid] * cosf(Param->Load_Theta[legid] * REG_TO_RADIN),
                         Param->Load_Rho[legid] * sinf(Param->Load_Theta[legid] * REG_TO_RADIN), Backflip_VMC.VMC_Cushion);
        }
        if ((t >= t5) && (wait_for_jump_flag == 2)) // 后空翻动作结束
        {
            wait_for_jump_flag = 0; // 等待第二次跳的标志位置0
            wait_for_jump = 0;
            memory_flag = 0;
            Jump_State = true;
        }
    }
    return Jump_State;
}

bool Frontflip_s(uint8_t legid, BackflipParam *Param, action_param *Action_Param, Euler *Euler_Angle, float t)
{
    bool Jump_State = false;
    if (legid <= 3 && legid >= 0) // 防数组越界
    {
        // 后空翻开始,keep状态节点
        float t0 = 0;

        // 下蹲蓄力过程,蓄力完节点
        float t1 = t0 + Param->Prep_Time[legid] + Param->Prep_Buffer_Time[legid];

        // 后腿(1,2)第一次起跳,直到身体转过75度左右节点
        float t2 = t1 + Param->Backflip_1_Time[legid];

        // 后腿(1,2)摆到与身体成一直线(90度左右),前腿摆过30度左右,进行第二次起跳(前腿0,3),再有一个缓冲节点
        float t3 = t2 + Param->Shift_Time[legid] + Param->Backflip_2_Time[legid];

        // 四条腿摆到预备落地的位置,准备落地并落地节点
        float t4 = t3 + Param->Swing_in_Air_Time[legid];

        // 落地缓冲,动作结束
        float t5 = t4 + Param->Land_Buffer_Time[legid];

        if (t <= t0) // 什么都不做
        {
            // 0 ~ t0
            // line函数在动作未开始时在原地停着因此运动始末点均为同一点,即足端初始位置点
            // 停留时间t0,采用VMC_Swing参数
            line(legid,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 t0, t - t0, &Backflip_VMC.VMC_Swing[legid]);
        }
        else if ((t > t0) && (t < t1)) // 四条腿下蹲,收腿起跳
        {
            if (t <= (t0 + Param->Prep_Time[legid]))
            {
                // t0 ~ (t0+Prep_Time)
                // line函数从Init_Pos到收腿目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
                // 运动时间Prep_Time,采用VMC_Swing参数
                line(legid,
                     Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Time[legid], t - t0, &Backflip_VMC.VMC_Swing[legid]);
            }
            else if (t > (t0 + Param->Prep_Time[legid]))
            {
                // 保持不动
                // line函数在收腿目标位置稍作停留,使身体稳定,因此运动始末点均为同一点,即预设起跳点
                // 停留时间Prep_Buffer_Time,采用VMC_Swing参数
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Buffer_Time[legid], t - t0 - Param->Prep_Time[legid], &Backflip_VMC.VMC_Swing[legid]);
            }
        }
        else if ((t > t1) && (t <= t2)) // 起跳
        {
            if (legid == 0 || legid == 3)
            {
                Change_Single_Pitch_State(Follow_Dog, 0);
                Change_Single_Pitch_State(Follow_Dog, 3);

                // 走直线,后腿起跳！
                // line函数从收腿位置到蹬腿起跳目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
                // 运动时间Backflip_1_Time,采用VMC_Swing参数
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Rho[legid] * cosf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Rho[legid] * sinf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Time[legid], t - t1, &Backflip_VMC.VMC_Swing[legid]);
            }
            if (legid == 1 || legid == 2)
            {
                // 前腿0和3随身体摆动自然摆动,保证腿和地面垂直
                Change_Single_Pitch_State(Follow_Ground, 1);
                Change_Single_Pitch_State(Follow_Ground, 2);
                line(legid,
                     Param->Prep_Rho[legid] * cosf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * cosf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     1, 1, &Backflip_VMC.VMC_Swing[legid]);
            }
        }
        else if ((t > t2) && (t <= t3)) // 腿摆到位置准备进行第二次前腿(0,3)的起跳
        {
            if (legid == 0 || legid == 3)
            {
                if (t <= (t2 + Param->Shift_Time[legid])) // 腿摆动到预备第二次起跳的位置
                {
                    // 后腿1和2摆到与身体成一直线
                    // 摆动时间Shift_Time
                    Change_Single_Pitch_State(Follow_Dog, 0);
                    Change_Single_Pitch_State(Follow_Dog, 3);
                    Bezier(legid,
                           Param->Backflip_1_Rho[legid] * cosf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                           Param->Backflip_1_Rho[legid] * sinf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                           Param->Shift_Rho[legid] * cosf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                           Param->Shift_Rho[legid] * sinf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                           -0.1f,
                           (Param->Backflip_1_Rho[legid] * cosf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN) + Param->Shift_Rho[legid] * cosf(Param->Shift_Theta[legid] * REG_TO_RADIN)) / 2.0f,
                           (Param->Backflip_1_Rho[legid] * sinf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN) + Param->Shift_Rho[legid] * sinf(Param->Shift_Theta[legid] * REG_TO_RADIN)) / 2.0f,
                           Param->Shift_Time[legid], t - t2, &Backflip_VMC.VMC_Swing[legid]);
                }
            }
            if (legid == 1 || legid == 2)
            {
                // 前腿0和3随身体摆动自然摆动,保证腿和地面垂直
                Change_Single_Pitch_State(Follow_Ground, 1);
                Change_Single_Pitch_State(Follow_Ground, 2);
                line(legid,
                     Param->Prep_Rho[legid] * cosf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * cosf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     Param->Shift_Time[legid], t - t2, &Backflip_VMC.VMC_Swing[legid]);
            }
        }

        // 俯仰角条件：当身体向前转过90度再跳,不然别跳(因为是往前转,所以拿Pitch跟-90度进行比较)
        if ((t > (t2 + Param->Shift_Time[legid])) && (fmodf((fabs(Euler_Angle->Pitch) + 180.0f), 360.0f) <= 90.0f))
        {
            if (wait_for_jump_flag == 0) // 第一次进入判断的时候刚刚起跳,俯仰角肯定不符合条件,且wait_for_jump_flag为0,此时记录一下时间,存到wait_for_jump里面
            {
                wait_for_jump = t;
                wait_for_jump_flag = 1;
            }
            Change_Single_Pitch_State(Follow_Ground, 1);
            Change_Single_Pitch_State(Follow_Ground, 2);

            // 什么都不做,保持位置
            if (legid == 0 || legid == 3)
            {
                line(legid,
                     Param->Shift_Rho[legid] * cosf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                     Param->Shift_Rho[legid] * sinf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                     Param->Shift_Rho[legid] * cosf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                     Param->Shift_Rho[legid] * sinf(Param->Shift_Theta[legid] * REG_TO_RADIN),
                     wait_for_jump, wait_for_jump, &Backflip_VMC.VMC_Swing[legid]);
            }
            if (legid == 1 || legid == 2)
            {
                line(legid,
                     Param->Prep_Rho[legid] * cosf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * cosf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf((Param->Prep_Theta[legid] - 180.0f) * REG_TO_RADIN),
                     wait_for_jump, wait_for_jump, &Backflip_VMC.VMC_Swing[legid]);
            }
            t = wait_for_jump; // 之后t = wait_for_jump肯定在(t2+Shift_Time)~t3之间,只要下面t2~t3中加一个标志位判断即可
        }
        else if ((t > (t2 + Param->Shift_Time[legid])) && (fmodf((fabs(Euler_Angle->Pitch) + 180.0f), 360.0f) > 90.0f))
        {
            if (wait_for_jump_flag == 1)
            {
                wait_for_jump = (t - wait_for_jump); // 现在wait_for_jump就是等待的delta_t
                wait_for_jump_flag = 2;
            }
            t = t - wait_for_jump;
        }

        if ((t > (t2 + Param->Shift_Time[legid])) && (t <= t3) && (wait_for_jump_flag == 2)) // 腿摆到位置,并且俯仰角符合条件,进行第二次前腿(0,3)的起跳
        {
            Change_Single_Pitch_State(Follow_Dog, 1);
            Change_Single_Pitch_State(Follow_Dog, 2);

            if (memory_flag == 0)
            {
                for (int i = 0; i <= 3; i++)
                {
                    Get_Now_Pos_Spd(i);
                    Action_Param->Init_Pos[i].x = leg[i].Foot.p.x;
                    Action_Param->Init_Pos[i].z = leg[i].Foot.p.z;
                }
                memory_flag = 1;
            }

            // 第二次前腿起跳
            // 走直线
            // line函数从收腿位置到蹬腿起跳目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
            // 运动时间Backflip_2_Time,采用VMC_Swing参数
            line(legid,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 Param->Backflip_2_Rho[legid] * cosf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                 Param->Backflip_2_Rho[legid] * sinf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                 Param->Backflip_2_Time[legid], t - (t2 + Param->Shift_Time[legid]), &Backflip_VMC.VMC_Swing[legid]);
        }
        else if ((t > t3) && (t <= t4) && (wait_for_jump_flag == 2)) // 四条腿摆到预备落地的位置,落地
        {                                                            // t3 ~ t4
            // Bezier函数从第二次跳完后的腿的位置到落地时预设目标位置,目标位置采用极坐标(ρ,θ),θ为单条腿正方向(水平向左)和原点到足端连线的夹角
            // 运动时间Swing_in_Air_Time,采用VMC_Swing参数
            Bezier(legid,
                   Param->Backflip_2_Rho[legid] * cosf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                   Param->Backflip_2_Rho[legid] * sinf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                   Param->Load_Rho[legid] * cosf(Param->Load_Theta[legid] * REG_TO_RADIN),
                   Param->Load_Rho[legid] * sinf(Param->Load_Theta[legid] * REG_TO_RADIN),
                   -0.1f,
                   (Param->Backflip_2_Rho[legid] * cosf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN) + Param->Load_Rho[legid] * cosf(Param->Load_Theta[legid] * REG_TO_RADIN)) / 2.0f,
                   (Param->Backflip_2_Rho[legid] * sinf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN) + Param->Load_Rho[legid] * sinf(Param->Load_Theta[legid] * REG_TO_RADIN)) / 2.0f,
                   Param->Swing_in_Air_Time[legid],
                   t - t3, &Backflip_VMC.VMC_Swing[legid]);
        }

        else if ((t > t4) && (t <= t5) && (wait_for_jump_flag == 2)) // 阻尼减震
        {
            Keep_One_Leg(legid,
                         Param->Load_Rho[legid] * cosf(Param->Load_Theta[legid] * REG_TO_RADIN),
                         Param->Load_Rho[legid] * sinf(Param->Load_Theta[legid] * REG_TO_RADIN), Backflip_VMC.VMC_Cushion);
        }
        if ((t >= t5) && (wait_for_jump_flag == 2)) // 后空翻动作结束
        {
            wait_for_jump_flag = 0; // 等待第二次跳的标志位置0
            wait_for_jump = 0;
            memory_flag = 0;
            Jump_State = true;
        }
    }
    return Jump_State;
}

void backflip_enter()
{
    SetDogChangeAble(DISABLE); // 动作完成前禁止切换状态
    GetTick();
    tick_start = Time_Points_Param.Now_Tick; // 获取当前时间
    for (int i = 0; i <= 3; i++)
    {
        Get_Now_Pos_Spd(i);
        Euler_Update(&Euler_Angle);
        Coordinate_Transform(&leg[i].Foot.p, &Action_Param.Init_Pos[i], 0, &leg[i], i);
    }
    Change_Dog_Pitch_State(Follow_Dog);
    memory_flag = 0;
}

void backflip_run()
{
    GetTick();
    float t = Time_Points_Param.Now_Tick - tick_start;

    bool finish_state = false;
    if (Get_Now_Ctrl_State() == Pos_Ctrl)
    {
        Change_Ctrl_State(Force_Ctrl);
    }
    if (Backfliptype == First_Flip)
    {
        finish_state = Backflip_s(0, &Backflip_Param[Backfliptype], &Action_Param, &Euler_Angle, t);
        Backflip_s(1, &Backflip_Param[Backfliptype], &Action_Param, &Euler_Angle, t);
        Backflip_s(2, &Backflip_Param[Backfliptype], &Action_Param, &Euler_Angle, t);
        Backflip_s(3, &Backflip_Param[Backfliptype], &Action_Param, &Euler_Angle, t);
    }

    if (Backfliptype == Second_Flip)
    {
        finish_state = Frontflip_s(0, &Backflip_Param[Backfliptype], &Action_Param, &Euler_Angle, t);
        Frontflip_s(1, &Backflip_Param[Backfliptype], &Action_Param, &Euler_Angle, t);
        Frontflip_s(2, &Backflip_Param[Backfliptype], &Action_Param, &Euler_Angle, t);
        Frontflip_s(3, &Backflip_Param[Backfliptype], &Action_Param, &Euler_Angle, t);
    }
    if (finish_state == true)
        SetDogChangeAble(ENABLE); // 所有腿的跳跃动作结束后允许跳出
}

void backflip_exit()
{
    if (Backfliptype == First_Flip)
        Backfliptype = Second_Flip;

    else if (Backfliptype == Second_Flip)
        Backfliptype = First_Flip;
}

BackflipType Backfliptype = First_Flip;
BackflipParam Backflip_Param[2] = {
    {
        // 后空翻(后腿先跳,前腿后跳,总共跳两次,前腿变后腿)
        // 前腿(腿0腿3)的Backflip_1和Shift过程的距离和角度参数没有写到函数里面,不用改

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        .Prep_Rho = {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Prep_Theta = {115, 90.01, 89.99, 65},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        .Prep_Time = {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        .Prep_Buffer_Time = {0.1, 0.1, 0.1, 0.1},

        // Backflip_1_Rho[4]; 第一次起跳时关节电机到足端的距离,
        .Backflip_1_Rho = {65535, 0.4, 0.4, 65535}, // 0,3还是收腿状态,1,2蹬腿

        // Backflip_1_Theta[4]; 第一次起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Backflip_1_Theta = {65535, 90, 90, 65535}, // 1,2腿向下蹬,给90度；0,3摆动到距离水平位置15度左右

        // Backflip_1_Time[4]; 第一次起跳时12蹬腿动作用的时间
        .Backflip_1_Time = {0.1, 0.1, 0.1, 0.1},

        // Shift_Rho[4]; 起跳后后腿在空中摆动到与身体成一条线时关节电机到足端的距离
        .Shift_Rho = {65535, 0.2, 0.2, 65535}, // 0,3还是收腿状态,准备第二次起跳,1,2收起来一些

        // Shift_Theta[4]; 起跳后后腿在空中摆动到与身体成一条线时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Shift_Theta = {65535, 180, 0, 65535}, // 0,3摆到超过身体直线15度左右,1,2与身体成一直线

        // Shift_Time[4]; 起跳后后腿在空中摆动到与身体成一条线用的时间
        .Shift_Time = {0.1, 0.1, 0.1, 0.1},

        // Backflip_2_Rho[4]; 第二次起跳时关节电机到足端的距离
        .Backflip_2_Rho = {0.4, 0.2, 0.2, 0.4}, // 0,3蹬腿

        // Backflip_2_Theta[4]; 第二次起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Backflip_2_Theta = {-1, 180, 0, 181}, //

        // Backflip_2_Time[4]; 第二次起跳时蹬腿动作用的时间
        .Backflip_2_Time = {0.2, 0.2, 0.2, 0.2},

        // Load_Rho[4]; 着陆时关节电机到足端的距离
        .Load_Rho = {0.18, 0.18, 0.18, 0.18},

        // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Load_Theta = {-90, 270, -90, 270},

        // Swing_in_Air_Time[4]; 第二次起跳完成后后腿的回摆时间,腿摆回去以后就落地了
        .Swing_in_Air_Time = {0.05, 0.05, 0.05, 0.05},

        // Land_Buffer_Time[4]; 落地缓冲时间
        .Land_Buffer_Time = {1.5, 1.5, 1.5, 1.5},

        // Std_Prep_Theta[4];  标准角度
        .Std_Prep_Theta = {90, 90, 90, 90},

        // Std_Backflip_1_Theta[4];  标准角度
        .Std_Backflip_1_Theta = {15, 90, 90, 165},

        // Std_Backflip_2_Theta[4]; // 标准角度
        .Std_Backflip_2_Theta = {-15, 180, 0, -165},

        // Std_Shift_Theta[4];  标准角度
        .Std_Shift_Theta = {-15, 180, 0, -165},

        // Std_Load_Theta[4]; // 标准角度
        .Std_Load_Theta = {-90, -90, -90, -90},
    },
    {
        // 新版测试,翻回来
        // 后空翻(后腿先跳,前腿后跳,总共跳两次,前腿变后腿)
        // 前腿(腿0腿3)的Backflip_1和Shift过程的距离和角度参数没有写到函数里面,不用改

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        .Prep_Rho = {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Prep_Theta = {-90, 295, -115, 270},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        .Prep_Time = {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        .Prep_Buffer_Time = {0.1, 0.1, 0.1, 0.1},

        // Backflip_1_Rho[4]; 第一次起跳时关节电机到足端的距离,
        .Backflip_1_Rho = {0.40, 65535, 65535, 0.40}, // 0,3还是收腿状态,1,2蹬腿

        // Backflip_1_Theta[4]; 第一次起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Backflip_1_Theta = {-90, 65535, 65535, 270}, // 1,2腿向下蹬,给90度；0,3摆动到距离水平位置15度左右

        // Backflip_1_Time[4]; 第一次起跳时12蹬腿动作用的时间
        .Backflip_1_Time = {0.1, 0.1, 0.1, 0.1},

        // Shift_Rho[4]; 起跳后后腿在空中摆动到与身体成一条线时关节电机到足端的距离
        .Shift_Rho = {0.2, 65535, 65535, 0.2}, // 0,3还是收腿状态,准备第二次起跳,1,2收起来一些

        // Shift_Theta[4]; 起跳后后腿在空中摆动到与身体成一条线时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Shift_Theta = {0, 65535, 65535, 180}, // 0,3摆到超过身体直线15度左右,1,2与身体成一直线

        // Shift_Time[4]; 起跳后后腿在空中摆动到与身体成一条线用的时间
        .Shift_Time = {0.1, 0.1, 0.1, 0.1},

        // Backflip_2_Rho[4]; 第二次起跳时关节电机到足端的距离
        //    .Backflip_2_Rho = {0.2, 0.3, 0.3, 0.2}, 近跳
        .Backflip_2_Rho = {0.2, 0.4, 0.4, 0.2},
        // Backflip_2_Theta[4]; 第二次起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Backflip_2_Theta = {0, 180, 0, 180}, //

        // Backflip_2_Time[4]; 第二次起跳时蹬腿动作用的时间
        .Backflip_2_Time = {0.2, 0.2, 0.2, 0.2},

        // Load_Rho[4]; 着陆时关节电机到足端的距离
        .Load_Rho = {0.18, 0.18, 0.18, 0.18},

        // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Load_Theta = {90, 90, 90, 90},

        // Swing_in_Air_Time[4]; 第二次起跳完成后后腿的回摆时间,腿摆回去以后就落地了
        .Swing_in_Air_Time = {0.1, 0.1, 0.1, 0.1},

        // Land_Buffer_Time[4]; 落地缓冲时间
        .Land_Buffer_Time = {1.5, 1.5, 1.5, 1.5},
    },
    /*
    {

        // 时间长的测试
        // 后空翻(后腿先跳,前腿后跳,总共跳两次,前腿变后腿)
        // 前腿(腿0腿3)的Backflip_1和Shift过程的距离和角度参数没有写到函数里面,不用改

        // Prep_Rho[4] 蓄力时关节电机到足端的距离
        .Prep_Rho = {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Prep_Theta = {115, 90.01, 89.99, 65},

        // Prep_Time[4]; 蓄力时收腿动作用的时间
        .Prep_Time = {4, 4, 4, 4},

        // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
        .Prep_Buffer_Time = {0.1, 0.1, 0.1, 0.1},

        // Backflip_1_Rho[4]; 第一次起跳时关节电机到足端的距离,
        .Backflip_1_Rho = {65535, 0.40, 0.40, 65535}, // 0,3还是收腿状态,1,2蹬腿

        // Backflip_1_Theta[4]; 第一次起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Backflip_1_Theta = {65535, 90, 90, 65535}, // 1,2腿向下蹬,给90度；0,3摆动到距离水平位置15度左右

        // Backflip_1_Time[4]; 第一次起跳时12蹬腿动作用的时间
        .Backflip_1_Time = {1, 1, 1, 1},

        // Shift_Rho[4]; 起跳后后腿在空中摆动到与身体成一条线时关节电机到足端的距离
        .Shift_Rho = {65535, 0.200, 0.200, 65535}, // 0,3还是收腿状态,准备第二次起跳,1,2收起来一些

        // Shift_Theta[4]; 起跳后后腿在空中摆动到与身体成一条线时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Shift_Theta = {65535, 180, 0, 65535}, // 0,3摆到超过身体直线15度左右,1,2与身体成一直线

        // Shift_Time[4]; 起跳后后腿在空中摆动到与身体成一条线用的时间
        .Shift_Time = {1, 1, 1, 1},

        // Backflip_2_Rho[4]; 第二次起跳时关节电机到足端的距离
        .Backflip_2_Rho = {0.390, 0.200, 0.200, 0.390}, // 0,3蹬腿

        // Backflip_2_Theta[4]; 第二次起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Backflip_2_Theta = {-1, 180, 0, 181}, //

        // Backflip_2_Time[4]; 第二次起跳时蹬腿动作用的时间
        .Backflip_2_Time = {2, 2, 2, 2},

        // Load_Rho[4]; 着陆时关节电机到足端的距离
        .Load_Rho = {0.18, 0.18, 0.18, 0.18},

        // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
        .Load_Theta = {-90, 270, -90, 270},

        // Swing_in_Air_Time[4]; 第二次起跳完成后后腿的回摆时间,腿摆回去以后就落地了
        .Swing_in_Air_Time = {1, 1, 1, 1},

        // Land_Buffer_Time[4]; 落地缓冲时间
        .Land_Buffer_Time = {1.5, 1.5, 1.5, 1.5},

        // Std_Prep_Theta[4];  标准角度
        .Std_Prep_Theta = {90, 90, 90, 90},

        // Std_Backflip_1_Theta[4];  标准角度
        .Std_Backflip_1_Theta = {15, 90, 90, 165},

        // Std_Backflip_2_Theta[4]; // 标准角度
        .Std_Backflip_2_Theta = {-15, 180, 0, -165},

        // Std_Shift_Theta[4];  标准角度
        .Std_Shift_Theta = {-15, 180, 0, -165},

        // Std_Load_Theta[4]; // 标准角度
        .Std_Load_Theta = {-90, -90, -90, -90},
    },
    */

    /*
    {
            // 专门跃高栏的参数
            // 后空翻(后腿先跳,前腿后跳,总共跳两次,前腿变后腿)
            // 前腿(腿0腿3)的Backflip_1和Shift过程的距离和角度参数没有写到函数里面,不用改

            // Prep_Rho[4] 蓄力时关节电机到足端的距离
            .Prep_Rho = {0.15, 0.15, 0.15, 0.15},

            // Prep_Theta[4]; 蓄力时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
            .Prep_Theta = {115, 90.01, 89.99, 65},

            // Prep_Time[4]; 蓄力时收腿动作用的时间
            .Prep_Time = {0.4, 0.4, 0.4, 0.4},

            // Prep_Buffer_Time[4]; 蓄力时收腿完成后的缓冲时间,收腿以后不直接跳是因为要保证狗身有时间缓冲到稳定状态
            .Prep_Buffer_Time = {0.1, 0.1, 0.1, 0.1},

            // Backflip_1_Rho[4]; 第一次起跳时关节电机到足端的距离,
            .Backflip_1_Rho = {65535, 0.40, 0.40, 65535}, // 0,3还是收腿状态,1,2蹬腿

            // Backflip_1_Theta[4]; 第一次起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
            .Backflip_1_Theta = {65535, 95, 85, 65535}, // 1,2腿向下蹬,给90度；0,3摆动到距离水平位置15度左右

            // Backflip_1_Time[4]; 第一次起跳时12蹬腿动作用的时间
            .Backflip_1_Time = {0.1, 0.1, 0.1, 0.1},

            // Shift_Rho[4]; 起跳后后腿在空中摆动到与身体成一条线时关节电机到足端的距离
            .Shift_Rho = {65535, 0.200, 0.200, 65535}, // 0,3还是收腿状态,准备第二次起跳,1,2收起来一些

            // Shift_Theta[4]; 起跳后后腿在空中摆动到与身体成一条线时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
            .Shift_Theta = {65535, 180, 0, 65535}, // 0,3摆到超过身体直线15度左右,1,2与身体成一直线

            // Shift_Time[4]; 起跳后后腿在空中摆动到与身体成一条线用的时间
            .Shift_Time = {0.1, 0.1, 0.1, 0.1},

            // Backflip_2_Rho[4]; 第二次起跳时关节电机到足端的距离
            .Backflip_2_Rho = {0.390, 0.200, 0.200, 0.390}, // 0,3蹬腿

            // Backflip_2_Theta[4]; 第二次起跳时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
            .Backflip_2_Theta = {-1, 180, 0, 181}, //

            // Backflip_2_Time[4]; 第二次起跳时蹬腿动作用的时间
            .Backflip_2_Time = {0.2, 0.2, 0.2, 0.2},

            // Load_Rho[4]; 着陆时关节电机到足端的距离
            .Load_Rho = {0.18, 0.18, 0.18, 0.18},

            // Load_Theta[4]; 着陆时关节电机到足端的连线与X轴正向的夹角0,逆时针为正
            .Load_Theta = {-90, 270, -90, 270},

            // Swing_in_Air_Time[4]; 第二次起跳完成后后腿的回摆时间,腿摆回去以后就落地了
            .Swing_in_Air_Time = {0.1, 0.1, 0.1, 0.1},

            // Land_Buffer_Time[4]; 落地缓冲时间
            .Land_Buffer_Time = {1.5, 1.5, 1.5, 1.5},

            // Std_Prep_Theta[4];  标准角度
            .Std_Prep_Theta = {90, 90, 90, 90},

            // Std_Backflip_1_Theta[4];  标准角度
            .Std_Backflip_1_Theta = {15, 90, 90, 165},

            // Std_Backflip_2_Theta[4]; // 标准角度
            .Std_Backflip_2_Theta = {-15, 180, 0, -165},

            // Std_Shift_Theta[4];  标准角度
            .Std_Shift_Theta = {-15, 180, 0, -165},

            // Std_Load_Theta[4]; // 标准角度
            .Std_Load_Theta = {-90, -90, -90, -90},
    }*/
};
