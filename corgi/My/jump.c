#include "jump.h"
#include "A1_motor.h"
#include "timer.h"
#include "cmsis_os.h"
#include "walk.h"
#include "timer.h"
#include "delay_DWT.h"
#include "pos_ctrl.h"

static float tick_start = 0; // �����Ŀ�ʼʱ��(s)

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
        // x����Kp x����Kd x������ȫ΢��ϵ��
        // z����Kp z����Kd z������ȫ΢��ϵ��
        // ǰ��x ǰ��z

        //        .Boost_x = -91,
        //        .Boost_z = 250,

        .Boost_x = 0,
        .Boost_z = 0,
};

/* --------------------------------------��Ծ--------------------------------------------- */

///////**
////// * @brief �������ǵķ���Pitch���ӵ��Ƕ�������ȶ���,����ʱ��Pitch������
////// * @param Euler ŷ���ǽṹ��,���ڶ�ȡ������
////// * @param Jump_Param ��Ծ�ṹ��
////// * @warning ���Load_Theta��û���뵽�ܺõİ취��ܸ���ֵ,����ô����һ��
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

//////        // ���Load_Theta��û���뵽�ܺõİ취��ܸ���ֵ,����ô����һ��
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

    // ���Load_Theta��û���뵽�ܺõİ취��ܸ���ֵ,����ô����һ��
    Jump_Param->Load_Theta[0] = Jump_Param->Std_Load_Theta;
    Jump_Param->Load_Theta[1] = Jump_Param->Std_Load_Theta;
    Jump_Param->Load_Theta[2] = 180.0f - Jump_Param->Std_Load_Theta;
    Jump_Param->Load_Theta[3] = 180.0f - Jump_Param->Std_Load_Theta;
}

/**
 * @brief ��ǰ����,�ù���Զ��
 * @param legid
 * @param Jump_Param
 * @param start_time ��ǰ�����Ŀ�ʼʱ��
 * @param end_time ��ǰ�����Ľ���ʱ��
 * @param t ��ǰʱ��
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

/* ---------------------------------------��Ծ���̺���----------------------------------------- */

/*
t0 + Prep_Time + Prep_Buffer_Time + Jump_Time + Jump_Buffer_Time + Tuck_Time + Tuck_Buffer_Time + Swing_in_Air_Time + Land_Buffer_Time
    ׼������������    �����Ժ󻺳�    ��Ծ����ʱ��   ����֮�󻺳屣��     ����ʱ��    �����Ժ󻺳屣��
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
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
    {
        // ʱ�䰲��
        float t0 = 0;
        float t1 = t0 + Jump_Param->Prep_Time[legid] + Jump_Param->Prep_Buffer_Time[legid];
        float t2 = t1 + Jump_Param->Jump_Time[legid] + Jump_Param->Jump_Buffer_Time[legid];
        float t3 = t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid] + Jump_Param->Swing_in_Air_Time[legid];
        float t4 = t3 + Jump_Param->Land_Buffer_Time[legid];

        //        Jump_Boost(legid, Jump_Param, t1, t1 + Jump_Param->Jump_Time[legid], t);

        // ��Ծ����Start!
        if (t <= t0) // ʲô������
        {
            // 0 ~ t0
            // line�����ڶ���δ��ʼʱ��ԭ��ͣ������˶�ʼĩ���Ϊͬһ��,����˳�ʼλ�õ�
            // ͣ��ʱ��t0,����VMC_Swing����
            line(legid,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 t0, t, &Jump_VMC.VMC_Swing[legid]);
        }
        else if ((t > t0) && (t <= t1)) // ����׼������
        {
            if (t <= (t0 + Jump_Param->Prep_Time[legid]))
            {
                // t0 ~ (t0+Prep_Time)
                // line������Init_Pos������Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
                // �˶�ʱ��Prep_Time,����VMC_Swing����
                line(legid,
                     Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                     Jump_Param->Prep_Rho[legid] * cosf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * sinf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Time[legid], t - t0, &Jump_VMC.VMC_Swing[legid]);
            }
            else if (t > (t0 + Jump_Param->Prep_Time[legid]))
            {
                // (t0+Prep_Time) ~ t1
                // line����������Ŀ��λ������ͣ��,ʹ�����ȶ�,����˶�ʼĩ���Ϊͬһ��,��Ԥ��������
                // ͣ��ʱ��Prep_Buffer_Time,����VMC_Swing����
                line(legid,
                     Jump_Param->Prep_Rho[legid] * cosf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * sinf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * cosf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * sinf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Buffer_Time[legid], t - t0 - Jump_Param->Prep_Time[legid], &Jump_VMC.VMC_Swing[legid]);
            }
        }
        else if ((t > t1) && (t <= t2)) // ����
        {
            if (t <= (t1 + Jump_Param->Jump_Time[legid]))
            {
                // t1 ~ (t1+Jump_Time)
                // line����������λ�õ���������Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
                // �˶�ʱ��Jump_Time,����VMC_Swing����
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
                // line�����ڵ���Ŀ��λ������ͣ��,����˶�ʼĩ���Ϊͬһ��,��Ԥ��������
                // ͣ��ʱ��Jump_Buffer_Time,����VMC_Swing����
                line(legid,
                     Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Jump_Buffer_Time[legid], t - t1 - Jump_Param->Jump_Time[legid], &Jump_VMC.VMC_Swing[legid]);
            }
        }
        else if ((t > t2) && (t <= t3)) // ���а���
        {
            if (t <= (t2 + Jump_Param->Tuck_Time[legid])) // ��������
            {
                // t2 ~ (t2+Tuck_Time)
                // line�����ӵ�������λ�õ���������Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
                // �˶�ʱ��Tuck_Time,����VMC_Swing����
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
                // line����������Ŀ��λ������ͣ��,����˶�ʼĩ���Ϊͬһ��
                // ͣ��ʱ��Tuck_Buffer_Time,����VMC_Swing����
                line(legid,
                     Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Buffer_Time[legid], t - t2 - Jump_Param->Tuck_Time[legid], &Jump_VMC.VMC_Swing[legid]);
            }
            else if (t > (t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid])) // ׼����½
            {
                // (t2+Tuck_Time+Tuck_Buffer_Time) ~ t3
                // line�����ӿ�������λ�õ����ʱԤ��Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
                // �˶�ʱ��Swing_in_Air_Time,����VMC_Swing����
                line(legid,
                     Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Load_Rho[legid] * cosf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Load_Rho[legid] * sinf(Jump_Param->Load_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Swing_in_Air_Time[legid],
                     t - t2 - Jump_Param->Tuck_Time[legid] - Jump_Param->Tuck_Buffer_Time[legid], &Jump_VMC.VMC_Swing[legid]);
            }
        }
        else if (t > t3 && t <= t4) // �������
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

/* ---------------------------------------״̬��----------------------------------------- */

// ������Ծ����
void ChangeJumpType(JumpType x)
{
    Jumptype = x;
}

void jump_enter()
{
    SetDogChangeAble(DISABLE); // �������ǰ��ֹ�л�״̬
    ChangeJumpType(Jumptype);  // ������Ծ״̬
    GetTick();
    Change_Dog_Pitch_State(Follow_Ground);
    tick_start = Time_Points_Param.Now_Tick; // ��ȡ��ǰʱ��
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

    // ����������
    //    Pitch_Jump_Offest(&Euler_Angle, &Jump_Param[Jumptype]);

    bool finish_state = false;
    if (Get_Now_Ctrl_State() == Force_Ctrl)
    {
        finish_state = Jump_s(0, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s(1, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s(2, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s(3, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);

        if (finish_state == true)
            SetDogChangeAble(ENABLE); // �����ȵ���Ծ������������������
    }
    else if (Get_Now_Ctrl_State() == Pos_Ctrl)
    {
			  Set_All_Motor_Param(&gait_param.Posctrl_Stance);
        finish_state = Jump_s_pos(0, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s_pos(1, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s_pos(2, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);
        Jump_s_pos(3, &Jump_Param[Jumptype], &Action_Param, &Euler_Angle, t);

        if (finish_state == true)
            SetDogChangeAble(ENABLE); // �����ȵ���Ծ������������������
    }
}
/* -----------------------------------������ʼ��------------------------------------------ */

JumpParam Jump_Param[15] = {
    {
        // First_Step һ������

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.155, 0.155, 0.155, 0.155},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {120, 120, 60, 60},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.395, 0.395, 0.395, 0.395},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.4, 0.4, 0.4, 0.4},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {120, 120, 60, 60},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.121, 0.121, 0.121, 0.121},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0, 0, 0, 0},
				
        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.16, 0.16, 0.16, 0.16},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {89, 89, 91, 91},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.3, 0.3, 0.3, 0.3},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        // ���ʱ�䲻�ô���0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X���� ��ļн�0,��ʱ��Ϊ��
        {89, 89, 91, 91},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,
    },
    {
        // �ȳ����0.14667m �0.4048m

        // Second_Step �ڶ���̨����Ծ

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.155, 0.155, 0.155, 0.155},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {91, 120, 60, 89},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.3, 0.45, 0.45, 0.3},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.40, 0.40, 0.40, 0.40},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {91, 120, 60, 89},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.05, 0.12, 0.12, 0.05},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0, 0, 0, 0},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.16, 0.16, 0.16, 0.16},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {76, 76, 104, 104},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.3, 0.3, 0.3, 0.3},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        // ���ʱ�䲻�ô���0.15s
        {0.1, 0.1, 0.1, 0.1},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {75, 75, 105, 105},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,

    },
    {
        // Third_Step ������̨����Ծ

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.18, 0.18, 0.18, 0.18},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {91, 110, 70, 89},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.3, 0.5, 0.5, 0.3},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {91, 110, 70, 89},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.15, 0.15, 0.15, 0.15},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0, 0, 0, 0},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.16, 0.16, 0.16, 0.16},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {76, 76, 104, 104},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        // ���ʱ�䲻�ô���0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {90, 90, 90, 90},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,
    },
    {
        // �ȳ����0.14667m �0.4048m

        // Bridge ˫ľ����Ծ

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.2, 0.2, 0.2, 0.2},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {91, 75, 105, 89},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.3, 0.4, 0.4, 0.3},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.4, 0.2, 0.2, 0.4},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {90, 105, 75, 90},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.15, 0.15, 0.15, 0.15},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0, 0, 0, 0},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.2, 0.2, 0.2, 0.2},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {50, 105, 75, 130},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        // ���ʱ�䲻�ô���0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {50, 105, 75, 130},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,

    },
    {
        // �ȳ����0.14667m �0.4048m

        // Bridge2 ˫ľ����Ծ����

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.2, 0.18, 0.18, 0.2},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {91, 91, 89, 89},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.3, 0.4, 0.4, 0.3},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.2, 0.39, 0.39, 0.2},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {91, 110, 70, 89},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.15, 0.15, 0.15, 0.15},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0, 0, 0, 0},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.2, 0.39, 0.39, 0.2},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {90, 60, 120, 90},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        // ���ʱ�䲻�ô���0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {90, 60, 120, 90},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,

    },
    {
        // Ground_Forward ��ǰ��

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.40, 0.38, 0.37, 0.37},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.15, 0.15, 0.15, 0.15},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0, 0, 0, 0},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.16, 0.16, 0.16, 0.16},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {76, 76, 104, 104},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        // ���ʱ�䲻�ô���0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {90, 90, 90, 90},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,
    },
    {
        // GroundBack ƽ�������Ծ

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.39, 0.39, 0.39, 0.39},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.15, 0.15, 0.15, 0.15},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0, 0, 0, 0},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.16, 0.16, 0.16, 0.16},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {76, 76, 104, 104},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        // ���ʱ�䲻�ô���0.15s
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
        {0.2, 0.2, 0.2, 0.2},

        // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {90, 90, 90, 90},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1.5, 1.5, 1.5, 1.5},

        .Std_Prep_Theta = 100.0f,

        .Std_Jump_Theta = 100.0f,

        .Std_Tuck_Theta = 80.0f,

        .Std_Load_Theta = 80.0f,
    },
    /* �����û�иĽṹ���ʼֵ,���þ��ȸ�һ��
    {
        // GroundBack ƽ�������Ծ

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {75, 75, 105, 105},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.27, 0.27, 0.27, 0.27},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.59, 0.59, 0.59, 0.59},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {75, 75, 105, 105},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {75, 75, 105, 105},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; ��½ʱ�����λ��
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {2, 2, 2, 2},
    },
    {
        // GroundLeft ԭ����ת��Ծ

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.27, 0.27, 0.27, 0.27},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.4, 0.4, 0.4, 0.4},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {70, 70, 70, 70},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.25, 0.25, 0.25, 0.25},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0, 0, 0, 0},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {70, 70, 70, 70},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; ��½ʱ�����λ��
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1, 1, 1, 1},

    },
    {
        // GroundRight ԭ����ת��Ծ

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 110, 110},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.27, 0.27, 0.27, 0.27},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.4, 0.4, 0.4, 0.4},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 110, 110},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.25, 0.25, 0.25, 0.25},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0, 0, 0, 0},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 110, 110},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; ��½ʱ�����λ��
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1, 1, 1, 1},
    },
    {
        // GroundUpToStep ƽ�ص�̨��

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.2, 0.2, 0.2, 0.2},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.5, 0.5, 0.5, 0.5},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.18, 0.18, 0.18, 0.18},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0.02, 0.02, 0.02, 0.02},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; ��½ʱ�����λ��
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1, 1, 1, 1},
    },
    {
        // StepUpToStep ��̨�׵�̨��

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.27, 0.27, 0.27, 0.27},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.5, 0.59, 0.5, 0.59},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; ��½ʱ�����λ��
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1, 1, 1, 1},
    },
    {
        // StepUpToGround ��̨�׵�ƽ��

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.12, 0.12, 0.12, 0.12},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.35, 0.35, 0.35, 0.35},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.55, 0.55, 0.55, 0.55},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.34, 0.34, 0.34, 0.34},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.34, 0.34, 0.34, 0.34},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0, 0, 0, 0},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.15, 0.15, 0.15, 0.15},

        // Load_Pos[4]; ��½ʱ�����λ��

        .Load_Pos = {{0, 0, 0.14}, {0, 0, 0.34}, {0, 0, 0.34}, {0, 0, 0.14}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {0.5, 0.5, 0.5, 0.5},
    },
    {
        // StepDownToStep ��̨�׵�̨��

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.12, 0.12, 0.12, 0.12},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.35, 0.35, 0.35, 0.35},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.55, 0.55, 0.55, 0.55},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.34, 0.34, 0.34, 0.34},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.34, 0.34, 0.34, 0.34},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0, 0, 0, 0},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.15, 0.15, 0.15, 0.15},

        // Load_Pos[4]; ��½ʱ�����λ��
        .Load_Pos = {{0, 0, 0.14}, {0, 0, 0.34}, {0, 0, 0.34}, {0, 0, 0.14}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {0.5, 0.5, 0.5, 0.5},
    },
    {
        // StepDownToGround ��̨�׵�ƽ��

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.12, 0.12, 0.12, 0.12},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.35, 0.35, 0.35, 0.35},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.55, 0.55, 0.55, 0.55},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.34, 0.34, 0.34, 0.34},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0.1, 0.1, 0.1, 0.1},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.34, 0.34, 0.34, 0.34},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {110, 110, 70, 70},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0, 0, 0, 0},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.15, 0.15, 0.15, 0.15},

        // Load_Pos[4]; ��½ʱ�����λ��
        .Load_Pos = {{0, 0, 0.14}, {0, 0, 0.34}, {0, 0, 0.34}, {0, 0, 0.14}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {0.5, 0.5, 0.5, 0.5},
    },
    {
        // ClimbUp ����

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.18, 0.18, 0.18, 0.18},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {120, 120, 60, 60},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.5, 0.5, 0.5, 0.5},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.59, 0.59, 0.59, 0.59},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {120, 120, 60, 60},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.2, 0.2, 0.2, 0.2},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0.05, 0.05, 0.05, 0.05},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.18, 0.18, 0.18, 0.18},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {120, 120, 60, 60},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0, 0, 0, 0},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; ��½ʱ�����λ��
        .Load_Pos = {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1, 1, 1, 1},
    },

    {
        // ClimbDown ����

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {115, 115, 65, 65},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.5, 0.5, 0.5, 0.5},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0.3, 0.3, 0.3, 0.3},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {115, 115, 65, 65},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.18, 0.18, 0.18, 0.18},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0.07, 0.07, 0.07, 0.07},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.18, 0.18, 0.18, 0.18},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {115, 115, 65, 65},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.05, 0.05, 0.05, 0.05},

        // Load_Pos[4]; ��½ʱ�����λ��
        {{0, 0, 0.25}, {0, 0, 0.25}, {0, 0, 0.25}, {0, 0, 0.25}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1, 1, 1, 1},

    },
    {
        // JumpTest1 ������

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.59, 0.59, 0.59, 0.59},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {},

        // Load_Pos[4]; ��½ʱ�����λ��
        {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {1, 1, 1, 1},

        .prep_angle = {30, 75, 75, 30},
        .jump_angle = {70, 80, 80, 70},
        .prep_height = {12, 6, 6, 12},

        .T.prep_time_start = {0, 0.55, 0.55, 0},    // ����ÿ�ʼ����
        .T.prep_time_length = {0.5, 0.1, 0.1, 0.5}, // ����ʱ�䳤��
        .T.prep_time = {0.5, 0.5, 0.5, 0.5},        // ����ʱ�� (s)

        .T.jump_time_length = {0.16, 0.1, 0.1, 0.15}, // ����ʱ�䳤�� 0.05
        .T.jump_time_damp = {0.8, 0.8, 0.8, 0.8},     // ���������������,��ʱû��
        .T.jump_time = {0.25, 0.25, 0.25, 0.25},      // ����ʱ��

        .T.tuck_time = {0.07, 0.07, 0.07, 0.07},           // ����ʱ��
        .T.prep_launch_time_start = {0, 0, 0, 0},          //   ��ʼ����
        .T.prep_launch_time_length = {0.1, 0.1, 0.1, 0.1}, // ����ʱ�䳤��
        .T.prep_launch_time = {0.3, 0.3, 0.3, 0.3},        // ׼������İ���ʱ��(���Ͽ�ʼʱ��)

    },
    {
        // JumpTest2 ǰ�ȱ� (���ڿ���)

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {100, 100, 80, 80},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0, 0, 0, 0},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.59, 0.59, 0.59, 0.59},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {100, 100, 80, 80},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.05, 0.05, 0.05, 0.05},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0.2, 0.2, 0.2, 0.2},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {100, 100, 80, 80},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.05, 0.05, 0.05, 0.05},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.05, 0.05, 0.05, 0.05},

        // Load_Pos[4]; ��½ʱ�����λ��
        {{0, 0, 0.4}, {0, 0, 0.4}, {0, 0, 0.4}, {0, 0, 0.4}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {0.7, 0.7, 0.7, 0.7},

    },
    {
        // JumpTest3 ���ȱ� (���ڿ���)

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        {0.38, 0.38, 0.38, 0.38},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {102, 102, 78, 78},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.23, 0.23, 0.23, 0.23},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        {0, 0, 0, 0},

        // Jump_Rho[4]; ����ʱ�ؽڵ������˵ľ���
        {0.59, 0.59, 0.59, 0.59},

        // Jump_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {102, 102, 78, 78},

        // Jump_Time[4]; ����ʱ���ȶ����õ�ʱ��
        {0.05, 0.05, 0.05, 0.05},

        // Jump_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��
        {0.18, 0.18, 0.18, 0.18},

        // Tuck_Rho[4]; �������ڿ�������ʱ�ؽڵ������˵ľ���
        {0.15, 0.15, 0.15, 0.15},

        // Tuck_Theta[4]; �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        {102, 102, 78, 78},

        // Tuck_Time[4]; �������ڿ��������õ�ʱ��
        {0.05, 0.05, 0.05, 0.05},

        // Tuck_Buffer_Time[4]; �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
        {0.15, 0.15, 0.15, 0.15},

        // Swing_in_Air_Time[4]; ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        {0.1, 0.1, 0.1, 0.1},

        // Load_Pos[4]; ��½ʱ�����λ��
        {{0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}, {0, 0, 0.2}},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        {0.8, 0.8, 0.8, 0.8},
    },
    */
};
