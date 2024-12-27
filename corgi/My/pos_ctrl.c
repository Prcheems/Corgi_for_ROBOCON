#include "pos_ctrl.h"
#include "jump.h"

/**
 * @brief �����һ��ֱ��,λ��
 * @param legid �����ȵ�id,�ж�����ֻ����ֱ��
 * @param start_x,start_z ֱ��ʼ��
 * @param end_x,end_z ֱ��ĩ��
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @param Swing PD���ưڶ������
 */
void line_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float T, float t)
{
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
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
 * @brief �����һ������,λ��
 * @param legid �����ȵ�id,�ж�����ֻ����ֱ��
 * @param start_x,start_z ����ʼ��
 * @param end_x ����ĩ��
 * @param max_z ���߶���z����
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @param Swing PD���ưڶ������
 */
void Cycloid_Pos(uint8_t legid, float start_x, float start_z, float end_x, float max_z, float T, float t)
{
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
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
 * @brief ����߱���������,�ڶ���ʹ��,λ��
 * @param legid �����ȵ�id,�ж�����ֻ���߰���(�漰�ٶ�ȡ��ȡ��)
 * @param start_x,start_z ������ϵ������ʼ��
 * @param end_x,end_z ������ϵ������ĩ��
 * @param max_z ���ߵ����߶�,���������д0,��������ϵz�������Ϊ-,��������ϵz�Ḻ���Ϊ+
 * @param Delta_x,Delta_z ��������ϵ��,���������ߵ�ԭ������
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @param Swing PD���ưڶ������
 */
void Bezier_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t)
{
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
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
 * @brief ����߱���������,walk״̬֧����ʹ��,λ��
 * @param legid �����ȵ�id,�ж�����ֻ���߰���(�漰�ٶ�ȡ��ȡ��)
 * @param start_x,start_z ������ϵ������ʼ��
 * @param end_x,end_z ������ϵ������ĩ��
 * @param max_z ���ߵ����߶�,���������д0,��������ϵz�������Ϊ-,��������ϵz�Ḻ���Ϊ+
 * @param Delta_x,Delta_z ��������ϵ��,���������ߵ�ԭ������
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @param Stance ֧����֧�ŵĲ���
 */
void Bezier_Stance_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t)
{
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
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

// λ��
void gait_posctrl(uint8_t legid, GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t)
{
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
    {
        if (gait_param->Leg_State[legid] == Swing)
        {
            switch (legid) // ��2��3��x������ȡ��
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
                           0.5f * gait_param->term * gait_param->swing_percent, t); // ������2��3�ĳ�ʼλ�ñ���ȡ������������
                break;
            case 3:
                Bezier_Pos(legid,
                           Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                           -gait_param->step_length[3] / 2.0f, gait_param->stance_height,
                           gait_param->step_height,
                           0, gait_param->stance_height,
                           0.5f * gait_param->term * gait_param->swing_percent, t - 0.5f * gait_param->term); // ������2��3�ĳ�ʼλ�ñ���ȡ������������
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
    // ��Ϊ��̬�������˶�,��ʱ��t���̵�һ��������
    if (t >= gait_param->term)
    {
        float t_former = t;             // �ݴ�ԭ����ʱ��t
        t = fmodf(t, gait_param->term); // ��t��termȡ��,�õ�0~term��һ����
        // gait_param->Cycle_Index = round((t_former - t) / gait_param->term); // ����ӿ�ʼ��tʱ��һ�������˼�������,�������ڸ������Ŀ��λ��
    }
    gait_param->last_t = gait_param->t; // t�洢���ǵ�ǰʱ��,��gait_param->last_t��gait_param->t��Ϊ�˼�¼��һ�ν���walk_away����ʱ���ʱ��
    gait_param->t = t;

    SetDogChangeAble(DISABLE);

    // �ȵ�״̬�л� ʱ��
    // �����ǰʱ��gait_param->t��0`0.5����,����һ�̵�ʱ�����0.5����(����һ����һ����̬���ڵĽ�β),�л�һ��״̬
    if ((((gait_param->t) >= 0) && ((gait_param->t) < (0.5f * gait_param->term)) && ((gait_param->last_t) > (0.5f * gait_param->term))) ||
        (((gait_param->t) <= 0.001f) && ((gait_param->last_t <= 0.001f)))) // ��0��2�ڶ���,��1��3֧����
    {
        // �л�״̬
        gait_param->Leg_State[0] = Swing;
        gait_param->Leg_State[2] = Swing;
        gait_param->Leg_State[1] = Stance;
        gait_param->Leg_State[3] = Stance;

        SetDogChangeAble(ENABLE);

        // �л�״̬����һ�̻�ȡ��̬��ʼ��
        for (int i = 0; i <= 3; i++)
        {
            Get_Now_Pos_Spd(i);
            Euler_Update(Euler_Angle);
            Coordinate_Transform(&leg[i].Foot.p, &Action_Param->Init_Pos[i], -Euler_Angle->Pitch, &leg[i], i);
        }

        // ƫ���ǽ���
        Yaw_Pos_Crol(Euler_Angle, gait_param, gait_param->Yaw_max);

        // �����������޸Ĳ�̬
        Pitch_Pos_Offest(Euler_Angle, gait_param);
    }
    // �����ǰʱ��gait_param->t��0.5`1������,����һ�̵�ʱ��С��0.5����(����һ���ǰ����̬���ڵĽ�β),�л�һ��״̬
    else if ((((gait_param->t) >= (0.5f * gait_param->term) && ((gait_param->t) < (gait_param->term))) &&
              (((gait_param->last_t) > 0) && (gait_param->last_t) < (0.5f * gait_param->term))) ||
             ((gait_param->t) >= (0.5f * gait_param->term) && (gait_param->t) <= (0.5f * gait_param->term + 0.001) &&
              ((gait_param->last_t) >= (0.5f * gait_param->term)) && (gait_param->last_t) <= (0.5f * gait_param->term + 0.001))) // ��1��3�ڶ���,��0��2֧����
    {
        // �л�״̬
        gait_param->Leg_State[1] = Swing;
        gait_param->Leg_State[3] = Swing;
        gait_param->Leg_State[0] = Stance;
        gait_param->Leg_State[2] = Stance;

        SetDogChangeAble(ENABLE);

        // �л�״̬����һ�̻�ȡ��̬��ʼ��
        for (int i = 0; i <= 3; i++)
        {
            Get_Now_Pos_Spd(i);
            Euler_Update(Euler_Angle);
            Coordinate_Transform(&leg[i].Foot.p, &Action_Param->Init_Pos[i], -Euler_Angle->Pitch, &leg[i], i);
        }

        // ƫ���ǽ���
        Yaw_Pos_Crol(Euler_Angle, gait_param, gait_param->Yaw_max);

        // �����������޸Ĳ�̬
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
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
    {
        // ʱ�䰲��
        float t0 = 0;
        float t1 = t0 + Jump_Param->Prep_Time[legid] + Jump_Param->Prep_Buffer_Time[legid];
        float t2 = t1 + Jump_Param->Jump_Time[legid];
        float t3 = t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid] + Jump_Param->Swing_in_Air_Time[legid];
        float t4 = t3 + Jump_Param->Land_Buffer_Time[legid];

        // ��Ծ����Start!
        if (t <= t0) // ʲô������
        {
        }
        else if ((t > t0) && (t <= t1)) // ����׼������
        {
            Set_All_Motor_Param(&prep_param);
            line_Pos(legid,
                     Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                     Jump_Param->Prep_Rho[legid] * cosf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Rho[legid] * sinf(Jump_Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Jump_Param->Prep_Time[legid], t - t0);
        }
        else if ((t > t1) && (t <= t2)) // ����
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
        else if ((t > t2) && (t <= t3)) // ���а���
        {
            if (t <= (t2 + Jump_Param->Tuck_Time[legid])) // ��������
            {
                line_Pos(legid,
                         Jump_Param->Jump_Rho[legid] * cosf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Jump_Rho[legid] * sinf(Jump_Param->Jump_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Time[legid], t - t2);
            }
            else if ((t > (t2 + Jump_Param->Tuck_Time[legid])) && (t <= (t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid]))) // ʲô������
            {
                line_Pos(legid,
                         Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * cosf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Rho[legid] * sinf(Jump_Param->Tuck_Theta[legid] * REG_TO_RADIN),
                         Jump_Param->Tuck_Buffer_Time[legid], t - t2 - Jump_Param->Tuck_Time[legid]);
            }

            else if (t > (t2 + Jump_Param->Tuck_Time[legid] + Jump_Param->Tuck_Buffer_Time[legid])) // ׼����½
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
        else if (t > t3 && t <= t4) // �������
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
