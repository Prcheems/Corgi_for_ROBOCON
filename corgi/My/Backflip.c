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

static float tick_start = 0; // �����Ŀ�ʼʱ��(s)

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

float wait_for_jump = 0;        // �ȴ������Ƿ�������ʱ��¼���м�ʱ��
uint8_t wait_for_jump_flag = 0; // ��־λ,0��ʾ��δ��¼ʱ��,1��ʾ�Ѿ���¼ʱ��,����������δ��������,2��ʾ���Խ���ڶ�����Ծ
uint8_t memory_flag;

/* ---------------------------------------��շ����̺���----------------------------------------- */

/*
t0  +  Prep_Time  +  Prep_Buffer_Time  +  Backflip_1_Time  +  Shift_Time  +  Backflip_2_Time  +  Swing_in_Air_Time  +  Land_Buffer_Time
     ׼������������    �����Ժ󻺳�          ��������ʱ��   ���Ȱڵ�������ƽ��    ǰ������ʱ��     ���ǰ�Ȱڵ��ʵ�λ��     ��ػ���ʱ��
--------------t1---------------------
---------------------------------t2-------------------------
-----------------------------------------------------------t3-------------------------------------------------------
------------------------------------------------------------------------t4----------------------------------------------------------------
------------------------------------------------------------------------------------t5----------------------------------------------------------------------------------------

*/
bool Backflip_s(uint8_t legid, BackflipParam *Param, action_param *Action_Param, Euler *Euler_Angle, float t)
{
    bool Jump_State = false;
    if (legid <= 3 && legid >= 0) // ������Խ��
    {
        // ��շ���ʼ,keep״̬�ڵ�
        float t0 = 0;

        // �¶���������,������ڵ�
        float t1 = t0 + Param->Prep_Time[legid] + Param->Prep_Buffer_Time[legid];

        // ����(1,2)��һ������,ֱ������ת��75�����ҽڵ�
        float t2 = t1 + Param->Backflip_1_Time[legid];

        // ����(1,2)�ڵ��������һֱ��(90������),ǰ�Ȱڹ�30������,���еڶ�������(ǰ��0,3),����һ������ڵ�
        float t3 = t2 + Param->Shift_Time[legid] + Param->Backflip_2_Time[legid];

        // �����Ȱڵ�Ԥ����ص�λ��,׼����ز���ؽڵ�
        float t4 = t3 + Param->Swing_in_Air_Time[legid];

        // ��ػ���,��������
        float t5 = t4 + Param->Land_Buffer_Time[legid];

        if (t <= t0) // ʲô������
        {
            // 0 ~ t0
            // line�����ڶ���δ��ʼʱ��ԭ��ͣ������˶�ʼĩ���Ϊͬһ��,����˳�ʼλ�õ�
            // ͣ��ʱ��t0,����VMC_Swing����
            line(legid,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 t0, t - t0, &Backflip_VMC.VMC_Swing[legid]);
        }
        else if ((t > t0) && (t < t1)) // �������¶�,��������
        {
            if (t <= (t0 + Param->Prep_Time[legid]))
            {
                // t0 ~ (t0+Prep_Time)
                // line������Init_Pos������Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
                // �˶�ʱ��Prep_Time,����VMC_Swing����
                line(legid,
                     Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Time[legid], t - t0, &Backflip_VMC.VMC_Swing[legid]);
            }
            else if (t > (t0 + Param->Prep_Time[legid]))
            {
                // ���ֲ���
                // line����������Ŀ��λ������ͣ��,ʹ�����ȶ�,����˶�ʼĩ���Ϊͬһ��,��Ԥ��������
                // ͣ��ʱ��Prep_Buffer_Time,����VMC_Swing����
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Buffer_Time[legid], t - t0 - Param->Prep_Time[legid], &Backflip_VMC.VMC_Swing[legid]);
            }
        }
        else if ((t > t1) && (t <= t2)) // ����
        {
            if (legid == 1 || legid == 2)
            {
                Change_Single_Pitch_State(Follow_Dog, 1);
                Change_Single_Pitch_State(Follow_Dog, 2);

                // ��ֱ��,����������
                // line����������λ�õ���������Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
                // �˶�ʱ��Backflip_1_Time,����VMC_Swing����
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Rho[legid] * cosf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Rho[legid] * sinf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Time[legid], t - t1, &Backflip_VMC.VMC_Swing[legid]);
            }
            if (legid == 0 || legid == 3)
            {
                // ǰ��0��3������ڶ���Ȼ�ڶ�,��֤�Ⱥ͵��洹ֱ
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
        else if ((t > t2) && (t <= t3)) // �Ȱڵ�λ��׼�����еڶ���ǰ��(0,3)������
        {
            if (legid == 1 || legid == 2)
            {
                if (t <= (t2 + Param->Shift_Time[legid])) // �Ȱڶ���Ԥ���ڶ���������λ��
                {
                    // ����1��2�ڵ��������һֱ��
                    // �ڶ�ʱ��Shift_Time
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
                // ǰ��0��3������ڶ���Ȼ�ڶ�,��֤�Ⱥ͵��洹ֱ
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

        // ��������������������ǰת��90������,��Ȼ����(��Ϊ����ǰת,������Pitch��-90�Ƚ��бȽ�)
        if ((t > (t2 + Param->Shift_Time[legid])) && (fmodf(fabs(Euler_Angle->Pitch), 360.0f) <= 90.0f))
        {
            if (wait_for_jump_flag == 0) // ��һ�ν����жϵ�ʱ��ո�����,�����ǿ϶�����������,��wait_for_jump_flagΪ0,��ʱ��¼һ��ʱ��,�浽wait_for_jump����
            {
                wait_for_jump = t;
                wait_for_jump_flag = 1;
            }
            Change_Single_Pitch_State(Follow_Ground, 0);
            Change_Single_Pitch_State(Follow_Ground, 3);

            // ʲô������,����λ��
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
            t = wait_for_jump; // ֮��t = wait_for_jump�϶���(t2+Shift_Time)~t3֮��,ֻҪ����t2~t3�м�һ����־λ�жϼ���
        }
        else if ((t > (t2 + Param->Shift_Time[legid])) && (fmodf(fabs(Euler_Angle->Pitch), 360.0f) > 90.0f))
        {
            if (wait_for_jump_flag == 1)
            {
                wait_for_jump = (t - wait_for_jump); // ����wait_for_jump���ǵȴ���delta_t
                wait_for_jump_flag = 2;
            }
            t = t - wait_for_jump;
        }

        if ((t > (t2 + Param->Shift_Time[legid])) && (t <= t3) && (wait_for_jump_flag == 2)) // �Ȱڵ�λ��,���Ҹ����Ƿ�������,���еڶ���ǰ��(0,3)������
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

            // �ڶ���ǰ������
            // ��ֱ��
            // line����������λ�õ���������Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
            // �˶�ʱ��Backflip_2_Time,����VMC_Swing����
            line(legid,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 Param->Backflip_2_Rho[legid] * cosf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                 Param->Backflip_2_Rho[legid] * sinf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                 Param->Backflip_2_Time[legid], t - (t2 + Param->Shift_Time[legid]), &Backflip_VMC.VMC_Swing[legid]);
        }
        else if ((t > t3) && (t <= t4) && (wait_for_jump_flag == 2)) // �����Ȱڵ�Ԥ����ص�λ��,���
        {                                                            // t3 ~ t4
            // Bezier�����ӵڶ����������ȵ�λ�õ����ʱԤ��Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
            // �˶�ʱ��Swing_in_Air_Time,����VMC_Swing����
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

        else if ((t > t4) && (t <= t5) && (wait_for_jump_flag == 2)) // �������
        {
            Keep_One_Leg(legid,
                         Param->Load_Rho[legid] * cosf(Param->Load_Theta[legid] * REG_TO_RADIN),
                         Param->Load_Rho[legid] * sinf(Param->Load_Theta[legid] * REG_TO_RADIN), Backflip_VMC.VMC_Cushion);
        }
        if ((t >= t5) && (wait_for_jump_flag == 2)) // ��շ���������
        {
            wait_for_jump_flag = 0; // �ȴ��ڶ������ı�־λ��0
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
    if (legid <= 3 && legid >= 0) // ������Խ��
    {
        // ��շ���ʼ,keep״̬�ڵ�
        float t0 = 0;

        // �¶���������,������ڵ�
        float t1 = t0 + Param->Prep_Time[legid] + Param->Prep_Buffer_Time[legid];

        // ����(1,2)��һ������,ֱ������ת��75�����ҽڵ�
        float t2 = t1 + Param->Backflip_1_Time[legid];

        // ����(1,2)�ڵ��������һֱ��(90������),ǰ�Ȱڹ�30������,���еڶ�������(ǰ��0,3),����һ������ڵ�
        float t3 = t2 + Param->Shift_Time[legid] + Param->Backflip_2_Time[legid];

        // �����Ȱڵ�Ԥ����ص�λ��,׼����ز���ؽڵ�
        float t4 = t3 + Param->Swing_in_Air_Time[legid];

        // ��ػ���,��������
        float t5 = t4 + Param->Land_Buffer_Time[legid];

        if (t <= t0) // ʲô������
        {
            // 0 ~ t0
            // line�����ڶ���δ��ʼʱ��ԭ��ͣ������˶�ʼĩ���Ϊͬһ��,����˳�ʼλ�õ�
            // ͣ��ʱ��t0,����VMC_Swing����
            line(legid,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 t0, t - t0, &Backflip_VMC.VMC_Swing[legid]);
        }
        else if ((t > t0) && (t < t1)) // �������¶�,��������
        {
            if (t <= (t0 + Param->Prep_Time[legid]))
            {
                // t0 ~ (t0+Prep_Time)
                // line������Init_Pos������Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
                // �˶�ʱ��Prep_Time,����VMC_Swing����
                line(legid,
                     Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Time[legid], t - t0, &Backflip_VMC.VMC_Swing[legid]);
            }
            else if (t > (t0 + Param->Prep_Time[legid]))
            {
                // ���ֲ���
                // line����������Ŀ��λ������ͣ��,ʹ�����ȶ�,����˶�ʼĩ���Ϊͬһ��,��Ԥ��������
                // ͣ��ʱ��Prep_Buffer_Time,����VMC_Swing����
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Buffer_Time[legid], t - t0 - Param->Prep_Time[legid], &Backflip_VMC.VMC_Swing[legid]);
            }
        }
        else if ((t > t1) && (t <= t2)) // ����
        {
            if (legid == 0 || legid == 3)
            {
                Change_Single_Pitch_State(Follow_Dog, 0);
                Change_Single_Pitch_State(Follow_Dog, 3);

                // ��ֱ��,����������
                // line����������λ�õ���������Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
                // �˶�ʱ��Backflip_1_Time,����VMC_Swing����
                line(legid,
                     Param->Prep_Rho[legid] * cosf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Prep_Rho[legid] * sinf(Param->Prep_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Rho[legid] * cosf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Rho[legid] * sinf(Param->Backflip_1_Theta[legid] * REG_TO_RADIN),
                     Param->Backflip_1_Time[legid], t - t1, &Backflip_VMC.VMC_Swing[legid]);
            }
            if (legid == 1 || legid == 2)
            {
                // ǰ��0��3������ڶ���Ȼ�ڶ�,��֤�Ⱥ͵��洹ֱ
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
        else if ((t > t2) && (t <= t3)) // �Ȱڵ�λ��׼�����еڶ���ǰ��(0,3)������
        {
            if (legid == 0 || legid == 3)
            {
                if (t <= (t2 + Param->Shift_Time[legid])) // �Ȱڶ���Ԥ���ڶ���������λ��
                {
                    // ����1��2�ڵ��������һֱ��
                    // �ڶ�ʱ��Shift_Time
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
                // ǰ��0��3������ڶ���Ȼ�ڶ�,��֤�Ⱥ͵��洹ֱ
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

        // ��������������������ǰת��90������,��Ȼ����(��Ϊ����ǰת,������Pitch��-90�Ƚ��бȽ�)
        if ((t > (t2 + Param->Shift_Time[legid])) && (fmodf((fabs(Euler_Angle->Pitch) + 180.0f), 360.0f) <= 90.0f))
        {
            if (wait_for_jump_flag == 0) // ��һ�ν����жϵ�ʱ��ո�����,�����ǿ϶�����������,��wait_for_jump_flagΪ0,��ʱ��¼һ��ʱ��,�浽wait_for_jump����
            {
                wait_for_jump = t;
                wait_for_jump_flag = 1;
            }
            Change_Single_Pitch_State(Follow_Ground, 1);
            Change_Single_Pitch_State(Follow_Ground, 2);

            // ʲô������,����λ��
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
            t = wait_for_jump; // ֮��t = wait_for_jump�϶���(t2+Shift_Time)~t3֮��,ֻҪ����t2~t3�м�һ����־λ�жϼ���
        }
        else if ((t > (t2 + Param->Shift_Time[legid])) && (fmodf((fabs(Euler_Angle->Pitch) + 180.0f), 360.0f) > 90.0f))
        {
            if (wait_for_jump_flag == 1)
            {
                wait_for_jump = (t - wait_for_jump); // ����wait_for_jump���ǵȴ���delta_t
                wait_for_jump_flag = 2;
            }
            t = t - wait_for_jump;
        }

        if ((t > (t2 + Param->Shift_Time[legid])) && (t <= t3) && (wait_for_jump_flag == 2)) // �Ȱڵ�λ��,���Ҹ����Ƿ�������,���еڶ���ǰ��(0,3)������
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

            // �ڶ���ǰ������
            // ��ֱ��
            // line����������λ�õ���������Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
            // �˶�ʱ��Backflip_2_Time,����VMC_Swing����
            line(legid,
                 Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                 Param->Backflip_2_Rho[legid] * cosf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                 Param->Backflip_2_Rho[legid] * sinf(Param->Backflip_2_Theta[legid] * REG_TO_RADIN),
                 Param->Backflip_2_Time[legid], t - (t2 + Param->Shift_Time[legid]), &Backflip_VMC.VMC_Swing[legid]);
        }
        else if ((t > t3) && (t <= t4) && (wait_for_jump_flag == 2)) // �����Ȱڵ�Ԥ����ص�λ��,���
        {                                                            // t3 ~ t4
            // Bezier�����ӵڶ����������ȵ�λ�õ����ʱԤ��Ŀ��λ��,Ŀ��λ�ò��ü�����(��,��),��Ϊ������������(ˮƽ����)��ԭ�㵽������ߵļн�
            // �˶�ʱ��Swing_in_Air_Time,����VMC_Swing����
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

        else if ((t > t4) && (t <= t5) && (wait_for_jump_flag == 2)) // �������
        {
            Keep_One_Leg(legid,
                         Param->Load_Rho[legid] * cosf(Param->Load_Theta[legid] * REG_TO_RADIN),
                         Param->Load_Rho[legid] * sinf(Param->Load_Theta[legid] * REG_TO_RADIN), Backflip_VMC.VMC_Cushion);
        }
        if ((t >= t5) && (wait_for_jump_flag == 2)) // ��շ���������
        {
            wait_for_jump_flag = 0; // �ȴ��ڶ������ı�־λ��0
            wait_for_jump = 0;
            memory_flag = 0;
            Jump_State = true;
        }
    }
    return Jump_State;
}

void backflip_enter()
{
    SetDogChangeAble(DISABLE); // �������ǰ��ֹ�л�״̬
    GetTick();
    tick_start = Time_Points_Param.Now_Tick; // ��ȡ��ǰʱ��
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
        SetDogChangeAble(ENABLE); // �����ȵ���Ծ������������������
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
        // ��շ�(��������,ǰ�Ⱥ���,�ܹ�������,ǰ�ȱ����)
        // ǰ��(��0��3)��Backflip_1��Shift���̵ľ���ͽǶȲ���û��д����������,���ø�

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        .Prep_Rho = {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Prep_Theta = {115, 90.01, 89.99, 65},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        .Prep_Time = {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        .Prep_Buffer_Time = {0.1, 0.1, 0.1, 0.1},

        // Backflip_1_Rho[4]; ��һ������ʱ�ؽڵ������˵ľ���,
        .Backflip_1_Rho = {65535, 0.4, 0.4, 65535}, // 0,3��������״̬,1,2����

        // Backflip_1_Theta[4]; ��һ������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Backflip_1_Theta = {65535, 90, 90, 65535}, // 1,2�����µ�,��90�ȣ�0,3�ڶ�������ˮƽλ��15������

        // Backflip_1_Time[4]; ��һ������ʱ12���ȶ����õ�ʱ��
        .Backflip_1_Time = {0.1, 0.1, 0.1, 0.1},

        // Shift_Rho[4]; ����������ڿ��аڶ����������һ����ʱ�ؽڵ������˵ľ���
        .Shift_Rho = {65535, 0.2, 0.2, 65535}, // 0,3��������״̬,׼���ڶ�������,1,2������һЩ

        // Shift_Theta[4]; ����������ڿ��аڶ����������һ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Shift_Theta = {65535, 180, 0, 65535}, // 0,3�ڵ���������ֱ��15������,1,2�������һֱ��

        // Shift_Time[4]; ����������ڿ��аڶ����������һ�����õ�ʱ��
        .Shift_Time = {0.1, 0.1, 0.1, 0.1},

        // Backflip_2_Rho[4]; �ڶ�������ʱ�ؽڵ������˵ľ���
        .Backflip_2_Rho = {0.4, 0.2, 0.2, 0.4}, // 0,3����

        // Backflip_2_Theta[4]; �ڶ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Backflip_2_Theta = {-1, 180, 0, 181}, //

        // Backflip_2_Time[4]; �ڶ�������ʱ���ȶ����õ�ʱ��
        .Backflip_2_Time = {0.2, 0.2, 0.2, 0.2},

        // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
        .Load_Rho = {0.18, 0.18, 0.18, 0.18},

        // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Load_Theta = {-90, 270, -90, 270},

        // Swing_in_Air_Time[4]; �ڶ���������ɺ���ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        .Swing_in_Air_Time = {0.05, 0.05, 0.05, 0.05},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        .Land_Buffer_Time = {1.5, 1.5, 1.5, 1.5},

        // Std_Prep_Theta[4];  ��׼�Ƕ�
        .Std_Prep_Theta = {90, 90, 90, 90},

        // Std_Backflip_1_Theta[4];  ��׼�Ƕ�
        .Std_Backflip_1_Theta = {15, 90, 90, 165},

        // Std_Backflip_2_Theta[4]; // ��׼�Ƕ�
        .Std_Backflip_2_Theta = {-15, 180, 0, -165},

        // Std_Shift_Theta[4];  ��׼�Ƕ�
        .Std_Shift_Theta = {-15, 180, 0, -165},

        // Std_Load_Theta[4]; // ��׼�Ƕ�
        .Std_Load_Theta = {-90, -90, -90, -90},
    },
    {
        // �°����,������
        // ��շ�(��������,ǰ�Ⱥ���,�ܹ�������,ǰ�ȱ����)
        // ǰ��(��0��3)��Backflip_1��Shift���̵ľ���ͽǶȲ���û��д����������,���ø�

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        .Prep_Rho = {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Prep_Theta = {-90, 295, -115, 270},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        .Prep_Time = {0.4, 0.4, 0.4, 0.4},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        .Prep_Buffer_Time = {0.1, 0.1, 0.1, 0.1},

        // Backflip_1_Rho[4]; ��һ������ʱ�ؽڵ������˵ľ���,
        .Backflip_1_Rho = {0.40, 65535, 65535, 0.40}, // 0,3��������״̬,1,2����

        // Backflip_1_Theta[4]; ��һ������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Backflip_1_Theta = {-90, 65535, 65535, 270}, // 1,2�����µ�,��90�ȣ�0,3�ڶ�������ˮƽλ��15������

        // Backflip_1_Time[4]; ��һ������ʱ12���ȶ����õ�ʱ��
        .Backflip_1_Time = {0.1, 0.1, 0.1, 0.1},

        // Shift_Rho[4]; ����������ڿ��аڶ����������һ����ʱ�ؽڵ������˵ľ���
        .Shift_Rho = {0.2, 65535, 65535, 0.2}, // 0,3��������״̬,׼���ڶ�������,1,2������һЩ

        // Shift_Theta[4]; ����������ڿ��аڶ����������һ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Shift_Theta = {0, 65535, 65535, 180}, // 0,3�ڵ���������ֱ��15������,1,2�������һֱ��

        // Shift_Time[4]; ����������ڿ��аڶ����������һ�����õ�ʱ��
        .Shift_Time = {0.1, 0.1, 0.1, 0.1},

        // Backflip_2_Rho[4]; �ڶ�������ʱ�ؽڵ������˵ľ���
        //    .Backflip_2_Rho = {0.2, 0.3, 0.3, 0.2}, ����
        .Backflip_2_Rho = {0.2, 0.4, 0.4, 0.2},
        // Backflip_2_Theta[4]; �ڶ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Backflip_2_Theta = {0, 180, 0, 180}, //

        // Backflip_2_Time[4]; �ڶ�������ʱ���ȶ����õ�ʱ��
        .Backflip_2_Time = {0.2, 0.2, 0.2, 0.2},

        // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
        .Load_Rho = {0.18, 0.18, 0.18, 0.18},

        // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Load_Theta = {90, 90, 90, 90},

        // Swing_in_Air_Time[4]; �ڶ���������ɺ���ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        .Swing_in_Air_Time = {0.1, 0.1, 0.1, 0.1},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        .Land_Buffer_Time = {1.5, 1.5, 1.5, 1.5},
    },
    /*
    {

        // ʱ�䳤�Ĳ���
        // ��շ�(��������,ǰ�Ⱥ���,�ܹ�������,ǰ�ȱ����)
        // ǰ��(��0��3)��Backflip_1��Shift���̵ľ���ͽǶȲ���û��д����������,���ø�

        // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
        .Prep_Rho = {0.15, 0.15, 0.15, 0.15},

        // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Prep_Theta = {115, 90.01, 89.99, 65},

        // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
        .Prep_Time = {4, 4, 4, 4},

        // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
        .Prep_Buffer_Time = {0.1, 0.1, 0.1, 0.1},

        // Backflip_1_Rho[4]; ��һ������ʱ�ؽڵ������˵ľ���,
        .Backflip_1_Rho = {65535, 0.40, 0.40, 65535}, // 0,3��������״̬,1,2����

        // Backflip_1_Theta[4]; ��һ������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Backflip_1_Theta = {65535, 90, 90, 65535}, // 1,2�����µ�,��90�ȣ�0,3�ڶ�������ˮƽλ��15������

        // Backflip_1_Time[4]; ��һ������ʱ12���ȶ����õ�ʱ��
        .Backflip_1_Time = {1, 1, 1, 1},

        // Shift_Rho[4]; ����������ڿ��аڶ����������һ����ʱ�ؽڵ������˵ľ���
        .Shift_Rho = {65535, 0.200, 0.200, 65535}, // 0,3��������״̬,׼���ڶ�������,1,2������һЩ

        // Shift_Theta[4]; ����������ڿ��аڶ����������һ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Shift_Theta = {65535, 180, 0, 65535}, // 0,3�ڵ���������ֱ��15������,1,2�������һֱ��

        // Shift_Time[4]; ����������ڿ��аڶ����������һ�����õ�ʱ��
        .Shift_Time = {1, 1, 1, 1},

        // Backflip_2_Rho[4]; �ڶ�������ʱ�ؽڵ������˵ľ���
        .Backflip_2_Rho = {0.390, 0.200, 0.200, 0.390}, // 0,3����

        // Backflip_2_Theta[4]; �ڶ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Backflip_2_Theta = {-1, 180, 0, 181}, //

        // Backflip_2_Time[4]; �ڶ�������ʱ���ȶ����õ�ʱ��
        .Backflip_2_Time = {2, 2, 2, 2},

        // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
        .Load_Rho = {0.18, 0.18, 0.18, 0.18},

        // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
        .Load_Theta = {-90, 270, -90, 270},

        // Swing_in_Air_Time[4]; �ڶ���������ɺ���ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
        .Swing_in_Air_Time = {1, 1, 1, 1},

        // Land_Buffer_Time[4]; ��ػ���ʱ��
        .Land_Buffer_Time = {1.5, 1.5, 1.5, 1.5},

        // Std_Prep_Theta[4];  ��׼�Ƕ�
        .Std_Prep_Theta = {90, 90, 90, 90},

        // Std_Backflip_1_Theta[4];  ��׼�Ƕ�
        .Std_Backflip_1_Theta = {15, 90, 90, 165},

        // Std_Backflip_2_Theta[4]; // ��׼�Ƕ�
        .Std_Backflip_2_Theta = {-15, 180, 0, -165},

        // Std_Shift_Theta[4];  ��׼�Ƕ�
        .Std_Shift_Theta = {-15, 180, 0, -165},

        // Std_Load_Theta[4]; // ��׼�Ƕ�
        .Std_Load_Theta = {-90, -90, -90, -90},
    },
    */

    /*
    {
            // ר��Ծ�����Ĳ���
            // ��շ�(��������,ǰ�Ⱥ���,�ܹ�������,ǰ�ȱ����)
            // ǰ��(��0��3)��Backflip_1��Shift���̵ľ���ͽǶȲ���û��д����������,���ø�

            // Prep_Rho[4] ����ʱ�ؽڵ������˵ľ���
            .Prep_Rho = {0.15, 0.15, 0.15, 0.15},

            // Prep_Theta[4]; ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
            .Prep_Theta = {115, 90.01, 89.99, 65},

            // Prep_Time[4]; ����ʱ���ȶ����õ�ʱ��
            .Prep_Time = {0.4, 0.4, 0.4, 0.4},

            // Prep_Buffer_Time[4]; ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
            .Prep_Buffer_Time = {0.1, 0.1, 0.1, 0.1},

            // Backflip_1_Rho[4]; ��һ������ʱ�ؽڵ������˵ľ���,
            .Backflip_1_Rho = {65535, 0.40, 0.40, 65535}, // 0,3��������״̬,1,2����

            // Backflip_1_Theta[4]; ��һ������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
            .Backflip_1_Theta = {65535, 95, 85, 65535}, // 1,2�����µ�,��90�ȣ�0,3�ڶ�������ˮƽλ��15������

            // Backflip_1_Time[4]; ��һ������ʱ12���ȶ����õ�ʱ��
            .Backflip_1_Time = {0.1, 0.1, 0.1, 0.1},

            // Shift_Rho[4]; ����������ڿ��аڶ����������һ����ʱ�ؽڵ������˵ľ���
            .Shift_Rho = {65535, 0.200, 0.200, 65535}, // 0,3��������״̬,׼���ڶ�������,1,2������һЩ

            // Shift_Theta[4]; ����������ڿ��аڶ����������һ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
            .Shift_Theta = {65535, 180, 0, 65535}, // 0,3�ڵ���������ֱ��15������,1,2�������һֱ��

            // Shift_Time[4]; ����������ڿ��аڶ����������һ�����õ�ʱ��
            .Shift_Time = {0.1, 0.1, 0.1, 0.1},

            // Backflip_2_Rho[4]; �ڶ�������ʱ�ؽڵ������˵ľ���
            .Backflip_2_Rho = {0.390, 0.200, 0.200, 0.390}, // 0,3����

            // Backflip_2_Theta[4]; �ڶ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
            .Backflip_2_Theta = {-1, 180, 0, 181}, //

            // Backflip_2_Time[4]; �ڶ�������ʱ���ȶ����õ�ʱ��
            .Backflip_2_Time = {0.2, 0.2, 0.2, 0.2},

            // Load_Rho[4]; ��½ʱ�ؽڵ������˵ľ���
            .Load_Rho = {0.18, 0.18, 0.18, 0.18},

            // Load_Theta[4]; ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
            .Load_Theta = {-90, 270, -90, 270},

            // Swing_in_Air_Time[4]; �ڶ���������ɺ���ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
            .Swing_in_Air_Time = {0.1, 0.1, 0.1, 0.1},

            // Land_Buffer_Time[4]; ��ػ���ʱ��
            .Land_Buffer_Time = {1.5, 1.5, 1.5, 1.5},

            // Std_Prep_Theta[4];  ��׼�Ƕ�
            .Std_Prep_Theta = {90, 90, 90, 90},

            // Std_Backflip_1_Theta[4];  ��׼�Ƕ�
            .Std_Backflip_1_Theta = {15, 90, 90, 165},

            // Std_Backflip_2_Theta[4]; // ��׼�Ƕ�
            .Std_Backflip_2_Theta = {-15, 180, 0, -165},

            // Std_Shift_Theta[4];  ��׼�Ƕ�
            .Std_Shift_Theta = {-15, 180, 0, -165},

            // Std_Load_Theta[4]; // ��׼�Ƕ�
            .Std_Load_Theta = {-90, -90, -90, -90},
    }*/
};
