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
#define STEP_LENGTH 0.15f // �������0.15m
#define STEP_HEIGHT 0.05f

GaitParam gait_param = {
    .Leg_State = {Swing, Stance, Swing, Stance}, // �ȵ�״̬
    .Walk_State = First,                         // walk״̬
    .Dire = Front,
    .stance_height = STAND_HEIGHT,                                       // վ���߶�(m)
    .step_length = {STEP_LENGTH, STEP_LENGTH, STEP_LENGTH, STEP_LENGTH}, // ����(m)
    .step_height = STEP_HEIGHT,                                          // ����(m)
                                                                         //    .step_height_down = 0.05,                                            // ֧�����³�����
    .step_height_down = 0,                                               // ֧�����³�����
    .std_step_length = STEP_LENGTH,                                      // ��׼����,���㸳ֵ(m)

    .swing_percent = 0.99f, // �ڶ���ٷֱ�
    .term = 0.2f,          // ��̬����(s)
    .t = 0,                // ��ǰʱ��
    .last_t = 0,           // �ݴ��ϴε�ʱ��,����֧�Űڶ����л�

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
 * @brief �������ǵķ���Pitch���ӵ����ߺͲ�̬x����Գ������������ʱ�ȶ���
 * @param Euler ŷ���ǽṹ��,���ڶ�ȡ��ǰƫ���Ǻ�Ŀ��ƫ����
 * @param gait_param ��̬�ṹ��,��̬x����Գ���λ��
 * @warning ��������ֱ����step_length������-��,���step_length������std_step_lengthһֱ��������(��������Yaw_Pos_Crol)
 *          ʵ�ָ��µ�,��ô������һֱ��С�����׵ĳ̶�,���𻵽ṹ��������
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
 * @brief �������ǵķ���Yaw���ӵ������Ͻ���ƫ����
 * @param Euler ŷ���ǽṹ��,���ڶ�ȡ��ǰƫ���Ǻ�Ŀ��ƫ����
 * @param gait_param ��̬�ṹ��,�޸Ĳ���
 * @param Upper_Limit �ڲ�����׼std_step_length�����������󲽳�
 * @param Lower_Limit �ڲ�����׼std_step_length�������ɼ�С����
 */
void Yaw_Pos_Crol(Euler *Euler, GaitParam *gait_param, float Max_Changable_Step_Length)
{
    // �������error
    gait_param->VMC_Euler_Walk.err_x = Euler->Target_Yaw - Euler->Yaw;

    // ���㲻��ȫD��
    gait_param->VMC_Euler_Walk.differ_x = gait_param->VMC_Euler_Walk.Kd_x * (1 - gait_param->VMC_Euler_Walk.k_x) *
                                              (gait_param->VMC_Euler_Walk.err_x - gait_param->VMC_Euler_Walk.last_err_x) +
                                          gait_param->VMC_Euler_Walk.k_x * gait_param->VMC_Euler_Walk.last_differ_x;

    // �������
    float Delta_p = gait_param->VMC_Euler_Walk.Kp_x * gait_param->VMC_Euler_Walk.err_x + gait_param->VMC_Euler_Walk.differ_x;

    // ���ݲ���
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
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
    {
        if (gait_param->Leg_State[legid] == Swing)
        {
            switch (legid) // ��2��3��x������ȡ��
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
                       0.5f * gait_param->term * gait_param->swing_percent, t, &gait_param->VMC_Swing_Walk[2]); // ������2��3�ĳ�ʼλ�ñ���ȡ������������
                break;
            case 3:
                Bezier(legid,
                       Action_Param->Init_Pos[legid].x, Action_Param->Init_Pos[legid].z,
                       -gait_param->step_length[3] / 2.0f, gait_param->stance_height,
                       gait_param->step_height,
                       0, gait_param->stance_height,
                       0.5f * gait_param->term * gait_param->swing_percent, t - 0.5f * gait_param->term, &gait_param->VMC_Swing_Walk[3]); // ������2��3�ĳ�ʼλ�ñ���ȡ������������
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

// Ŀ��λ�ò��õ�ǰλ���ۼ�һ���������ǳ�ʼλ�ü�n������
// �����ĳһ����ƫ��Ԥ��λ�ù�����ô��
// дһ����λ��ԭʼkeep״̬�ĺ���
// ֧�������Ӧ��ʼ��λ��ʲô�ط�,����ʱ�ĵ���λ�Ʋ�?
void Walk_Away(GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t)
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
    if ((((gait_param->t) >= 0) && ((gait_param->t) < (0.5f * gait_param->term)) &&
         ((gait_param->last_t) > (0.5f * gait_param->term)) && ((gait_param->last_t) <= (gait_param->term))) ||
        (((gait_param->t) <= 0.001) && ((gait_param->last_t <= 0.001)))) // ��0��2�ڶ���,��1��3֧����
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
        /*
                if (gait_param->Walk_State == First)
                {
                    gait_param->step_length = gait_param->std_step_length / 2.0f;
                    gait_param->Walk_State = Later;
                }
        */
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
// �ⲿ�����޸�Walk�����Ľӿ�
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

static float tick_start = 0; // �����Ŀ�ʼʱ��(s)

void walk_enter()
{
    gait_param.t = 0;
    SetDogChangeAble(DISABLE);     // �������ǰ��ֹ�л�״̬
    gait_param.Walk_State = First; // ����walk״̬Ϊ��һ������
    Action_Param.Launch_Flag = Launch;
    GetTick();
    tick_start = Time_Points_Param.Now_Tick; // ��ȡ��ǰʱ��
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
        Walk_Away(&gait_param, &Action_Param, &Euler_Angle, t); // ����
    }
    if (Get_Now_Ctrl_State() == Pos_Ctrl)
    {
        Walk_Away_Posctrl(&gait_param, &Action_Param, &Euler_Angle, t); // λ��
    }
}

void walk_exit()
{
}
