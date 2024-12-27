#include "action.h"
#include "timer.h"
#include "imu.h"
#include "delay_DWT.h"
#include "pos_ctrl.h"
#include "A1_motor.h"
#include "pos_ctrl.h"

action_param Action_Param = {

    // ȫ״̬ͨ�ò���
    .Launch_Flag = Launch,
    .Init_Pos = {{0, 0, 0},
                 {0, 0, 0},
                 {0, 0, 0},
                 {0, 0, 0}},
    // Keep״̬����
    .Max_Diff = 0.01,
    .VMC_Stance_Keep = {
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
    },
    .Posctrl_keep = {0, 0.1, 2},
		.keep_type = 0,
		
    // Stand״̬����
//    .stand_height = {0.2, 0.2, 0.2, 0.2},
		.stand_height = {0.1475, 0.1475, 0.1475, 0.1475},

    .stand_tm = 2.0f,
    .VMC_Swing_Stand = {
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
    },
    .Posctrl_stand = {0, 0.1, 2},

    // Squat״̬����;
    .squat_x = {0.138, 0.138, -0.138, -0.138},
    .squat_z = {0.0512, 0.0512, 0.0512, 0.0512},
    .squat_tm = 2.0f,
    .VMC_Swing_Squat = {
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
        {400, 1000, 0.5, 4000, 4000, 0.5, 0, 0},
    },
    .Posctrl_Squat = {0, 0.1, 2},
};

static float tick_start = 0; // �����Ŀ�ʼʱ��(s)

/**
 * @brief ���ϵ�ʱ�Ŀ�ʼλ�õ�վ��λ��
 * @param t����ǰʱ��(s)
 * @param tm����������ʱ��(s)
 * @param h��վ���߶�(��˵���λ�ľ���)
 */
void Stand_Up(float t, float tm, float *h)
{
    if (Action_Param.Launch_Flag == Launch)
    {
        for (int i = 0; i <= 3; i++)
        {
            Get_Now_Pos_Spd(i);
            Euler_Update(&Euler_Angle);
            Coordinate_Transform(&leg[i].Foot.p, &Action_Param.Init_Pos[i], -Euler_Angle.Pitch, &leg[i], i);
        }
        Action_Param.Launch_Flag = Run;
    }
    else
    {
        if (t <= tm)
        {
            // line�����������0��1����ϵΪ��׼,�����Ҳ���ֻ��x����Ҫȡ��
            if (Get_Now_Ctrl_State() == Force_Ctrl) // ����
            {
                line(0, Action_Param.Init_Pos[0].x, Action_Param.Init_Pos[0].z, 0, h[0], tm, t, &Action_Param.VMC_Swing_Stand[0]);
                line(1, Action_Param.Init_Pos[1].x, Action_Param.Init_Pos[1].z, 0, h[1], tm, t, &Action_Param.VMC_Swing_Stand[1]);
                line(2, Action_Param.Init_Pos[2].x, Action_Param.Init_Pos[2].z, 0, h[2], tm, t, &Action_Param.VMC_Swing_Stand[2]);
                line(3, Action_Param.Init_Pos[3].x, Action_Param.Init_Pos[3].z, 0, h[3], tm, t, &Action_Param.VMC_Swing_Stand[3]);
            }
            else if (Get_Now_Ctrl_State() == Pos_Ctrl) // λ��
            {
                Bezier_Pos(0, Action_Param.Init_Pos[0].x, Action_Param.Init_Pos[0].z, 0, h[0], -0.05f, 0.068f, (0.05f + h[0]) / 2.0f, tm, t);
                Bezier_Pos(1, Action_Param.Init_Pos[1].x, Action_Param.Init_Pos[1].z, 0, h[1], -0.05f, 0.068f, (0.05f + h[0]) / 2.0f, tm, t);
                Bezier_Pos(2, Action_Param.Init_Pos[2].x, Action_Param.Init_Pos[2].z, 0, h[2], -0.05f, -0.068f, (0.05f + h[0]) / 2.0f, tm, t);
                Bezier_Pos(3, Action_Param.Init_Pos[3].x, Action_Param.Init_Pos[3].z, 0, h[3], -0.05f, -0.068f, (0.05f + h[0]) / 2.0f, tm, t);
            }
        }
        else
        {
            SetDogChangeAble(ENABLE);
            Action_Param.Launch_Flag = Finish;
        }
    }
}

/**
 * @brief �ӵ�ǰλ�õ��ϵ�ǰ�ĳ�ʼλ��
 * @param t����ǰʱ��(s)
 * @param tm����������ʱ��(s)
 * @param x,z����ʼ
 */
void Squat_Down(float t, float tm, float *x, float *z)
{
    if (Action_Param.Launch_Flag == Launch)
    {
        for (int i = 0; i <= 3; i++)
        {
            Get_Now_Pos_Spd(i);
            Euler_Update(&Euler_Angle);
            Coordinate_Transform(&leg[i].Foot.p, &Action_Param.Init_Pos[i], -Euler_Angle.Pitch, &leg[i], i);
        }
        Action_Param.Launch_Flag = Run;
    }
    else
    {
        if (t <= tm)
        {
            // line�����������0��1����ϵΪ��׼,�����Ҳ���ֻ��x����Ҫȡ��
            if (Get_Now_Ctrl_State() == Force_Ctrl) // ����
            {
                line(0, Action_Param.Init_Pos[0].x, Action_Param.Init_Pos[0].z, x[0], z[0], tm, t, &Action_Param.VMC_Swing_Squat[0]);
                line(1, Action_Param.Init_Pos[1].x, Action_Param.Init_Pos[1].z, x[1], z[1], tm, t, &Action_Param.VMC_Swing_Squat[1]);
                line(2, Action_Param.Init_Pos[2].x, Action_Param.Init_Pos[2].z, x[2], z[2], tm, t, &Action_Param.VMC_Swing_Squat[2]);
                line(3, Action_Param.Init_Pos[3].x, Action_Param.Init_Pos[3].z, x[3], z[3], tm, t, &Action_Param.VMC_Swing_Squat[3]);
            }
            else if (Get_Now_Ctrl_State() == Pos_Ctrl) // λ��
            {
                line_Pos(0, Action_Param.Init_Pos[0].x, Action_Param.Init_Pos[0].z, x[0], z[0], tm, t);
                line_Pos(1, Action_Param.Init_Pos[1].x, Action_Param.Init_Pos[1].z, x[1], z[1], tm, t);
                line_Pos(2, Action_Param.Init_Pos[2].x, Action_Param.Init_Pos[2].z, x[2], z[2], tm, t);
                line_Pos(3, Action_Param.Init_Pos[3].x, Action_Param.Init_Pos[3].z, x[3], z[3], tm, t);
            }
        }
        else
        {
            SetDogChangeAble(ENABLE);
            Action_Param.Launch_Flag = Finish;
        }
    }
}

struct crslg_time
{
		float dvd_time;
		float sit_time;
		float clc_time;
		float wtdw_time;
} Crslg_time = {1, 1, 1, 1}; // ʱ���ʼ��

void Cross_leg(float t)
{

//    /*
//    ˼·���ĸ�ʱ��Σ����ʱ��ڵ�
//    t0 = 0                                  ��ʼ
//    t1 = t0 + dvd_time (divide �ֿ�)         |
//    t2 = t1 + sit_time (�������̣�ǰ���쳤)  |
//    t3 = t2 + clc_time (click ��һ��̨��)   \|/
//    t4 = t3 + wtdw_time (withdraw ���ջ�)   ����
//    */
//    float t1 = Crslg_time.dvd_time;
//    float t2 = t1 + Crslg_time.sit_time;
//    float t3 = t2 + Crslg_time.clc_time;
//    float t4 = t3 + Crslg_time.wtdw_time;

//    if (Action_Param.Launch_Flag == Launch)
//    {
//        for (int i = 0; i <= 3; i++)
//        {
//            Get_Now_Pos_Spd(i);
//            Euler_Update(&Euler_Angle);
//            Coordinate_Transform(&leg[i].Foot.p, &Action_Param.Init_Pos[i], -Euler_Angle.Pitch, &leg[i], i);
//        }
//        Action_Param.Launch_Flag = Run;
//    }
//    else
//    {
//        if (t <= t1)
//        { // ��ǰ��ֿ�վ��
//            line(0, Action_Param.Init_Pos[0].x, Action_Param.Init_Pos[0].z, 0.1f,  Action_Param.Init_Pos[0].z, Crslg_time.dvd_time, t, &Action_Param.VMC_Swing_Stand[0]);
//            line(1, Action_Param.Init_Pos[1].x, Action_Param.Init_Pos[1].z, -0.1f, Action_Param.Init_Pos[1].z, Crslg_time.dvd_time, t, &Action_Param.VMC_Swing_Stand[1]);
//            line(2, Action_Param.Init_Pos[2].x, Action_Param.Init_Pos[2].z, -0.1f, Action_Param.Init_Pos[2].z, Crslg_time.dvd_time, t, &Action_Param.VMC_Swing_Stand[2]);
//            line(3, Action_Param.Init_Pos[3].x, Action_Param.Init_Pos[3].z, 0.1f,  Action_Param.Init_Pos[3].z, Crslg_time.dvd_time, t, &Action_Param.VMC_Swing_Stand[3]);
//        }
//        else if ((t > t1) && (t <= t2))
//        { // �������̣�ǰ���쳤
//            line(0, 0.1f,  Action_Param.Init_Pos[0].z, 0.1f,  0.2f,  Crslg_time.sit_time, t - t1, &Action_Param.VMC_Swing_Stand[0]);
//            line(1, -0.1f, Action_Param.Init_Pos[1].z, -0.1f, 0.15f, Crslg_time.sit_time, t - t1, &Action_Param.VMC_Swing_Stand[1]);
//            line(2, -0.1f, Action_Param.Init_Pos[2].z, -0.1f, 0.15f, Crslg_time.sit_time, t - t1, &Action_Param.VMC_Swing_Stand[2]);
//            line(3, 0.1f,  Action_Param.Init_Pos[3].z, 0.1f,  0.2f,  Crslg_time.sit_time, t - t1, &Action_Param.VMC_Swing_Stand[3]);
//        }
//        else if ((t > t2) && (t <= t3))
//        {
//            Bezier(0, 0.1f, 0.2f, 0.3f, 0.13f, 0.2f, 0, 0, Crslg_time.clc_time, t - t2, &Action_Param.VMC_Swing_Stand[0]);
//            // line(0, 0.1f, 0.2f, 0.1f, 0.2f, Crslg_time.clc_time, t - t2, &Action_Param.VMC_Swing_Stand[0]);
//            line(1, -0.1f, -0.1f, -0.1f, -0.1f, Crslg_time.clc_time, t - t2, &Action_Param.VMC_Swing_Stand[1]);
//            line(2, 0.1f, -0.1f, 0.1f, -0.1f, Crslg_time.clc_time, t - t2, &Action_Param.VMC_Swing_Stand[2]);
//            line(3, -0.1f, 0.2f, -0.1f, 0.2f, Crslg_time.clc_time, t - t2, &Action_Param.VMC_Swing_Stand[3]);
//        }
//        else if ((t > t3) && (t <= t4))
//        {
//            Bezier(0, 0.3f, 0.13f, Action_Param.Init_Pos[0].x, Action_Param.Init_Pos[0].z, 0.2f, 0, 0, Crslg_time.wtdw_time, t - t3, &Action_Param.VMC_Swing_Stand[0]);
//            // line(0, 0.1f, 0.2f, 0.1f, 0.2f, Crslg_time.wtdw_time, t - t3, &Action_Param.VMC_Swing_Stand[0]);
//            line(1, -0.1f, -0.1f, Action_Param.Init_Pos[1].x, Action_Param.Init_Pos[1].z, Crslg_time.wtdw_time, t - t3, &Action_Param.VMC_Swing_Stand[1]);
//            line(2, 0.1f, -0.1f, Action_Param.Init_Pos[2].x, Action_Param.Init_Pos[2].z, Crslg_time.wtdw_time, t - t3, &Action_Param.VMC_Swing_Stand[2]);
//            line(3, -0.1f, 0.2f, Action_Param.Init_Pos[3].x, Action_Param.Init_Pos[3].z, Crslg_time.wtdw_time, t - t3, &Action_Param.VMC_Swing_Stand[3]);
//        }
//        else
//        {
//            SetDogChangeAble(ENABLE);
//            Action_Param.Launch_Flag = Finish;
//        }
//    }
}


void Keep_All_Leg(float X0, float X1, float X2, float X3, float Z0, float Z1, float Z2, float Z3, PD *Stance)
{
    Point p[4] = {{X0, 0, Z0},
                  {X1, 0, Z1},
                  {X2, 0, Z2},
                  {X3, 0, Z3}};

    Coordinate_Transform(&p[0], &leg[0].Foot_Target.p, Euler_Angle.Pitch, &leg[0], 0);
    Coordinate_Transform(&p[1], &leg[1].Foot_Target.p, Euler_Angle.Pitch, &leg[1], 1);
    Coordinate_Transform(&p[2], &leg[2].Foot_Target.p, Euler_Angle.Pitch, &leg[2], 2);
    Coordinate_Transform(&p[3], &leg[3].Foot_Target.p, Euler_Angle.Pitch, &leg[3], 3);

    if (Get_Now_Ctrl_State() == Force_Ctrl)
    {
        for (int i = 0; i < 4; i++)
        {
            Get_Now_Pos_Spd(i);
            PD_Control(&Stance[i], &leg[i].Foot_Target.p, &leg[i].Foot.p, &leg[i].VMC_Output_Force);
            VMC_Jacobi_Matrix(&leg[i], &leg[i].const_param, &leg[i].VMC_Output_Force, &leg[i].Motor_Torque);
            Set_Motor_Target_Torque(&leg[i]);
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            Get_Now_Pos_Spd(i);
            Foot_Convert_To_Motor(&leg[i].const_param, &leg[i].Foot_Target, &leg[i].Pctrl.Joint_Target, &leg[i].Pctrl.Motor_Target);
            Set_Motor_Target_Pos(&leg[i]);
        }
    }
}

void Keep_One_Leg(uint8_t legid, float X, float Z, PD *Stance)
{
    Point p = {X, 0, Z};

    Coordinate_Transform(&p, &leg[legid].Foot_Target.p, Euler_Angle.Pitch, &leg[legid], legid);

//    if (Get_Now_Ctrl_State() == Force_Ctrl)
//    {
        Get_Now_Pos_Spd(legid);
        PD_Control(&Stance[legid], &leg[legid].Foot_Target.p, &leg[legid].Foot.p, &leg[legid].VMC_Output_Force);
        VMC_Jacobi_Matrix(&leg[legid], &leg[legid].const_param, &leg[legid].VMC_Output_Force, &leg[legid].Motor_Torque);
        Set_Motor_Target_Torque(&leg[legid]);
//    }
//    else
//    {
//        Get_Now_Pos_Spd(legid);
//        Foot_Convert_To_Motor(&leg[legid].const_param, &leg[legid].Foot_Target, &leg[legid].Pctrl.Joint_Target, &leg[legid].Pctrl.Motor_Target);
//        Set_Motor_Target_Pos(&leg[legid]);
//    }
}

bool Abs_Diff_Judge(float a, float b, float Max_Diff)
{
    bool state = false;
    float Diff = fabs(a - b);
    if (Diff >= Max_Diff)
        state = false;
    else if (Diff < Max_Diff)
        state = true;
    return state;
}

bool Judge_Keep_State(void)
{
    bool state = false;
    bool D0x = Abs_Diff_Judge(leg[0].Foot.p.x, 0, Action_Param.Max_Diff);
    bool D0z = Abs_Diff_Judge(leg[0].Foot.p.x, Action_Param.stand_height[0], Action_Param.Max_Diff);
    bool D1x = Abs_Diff_Judge(leg[1].Foot.p.x, 0, Action_Param.Max_Diff);
    bool D1z = Abs_Diff_Judge(leg[1].Foot.p.x, Action_Param.stand_height[1], Action_Param.Max_Diff);
    bool D2x = Abs_Diff_Judge(leg[2].Foot.p.x, 0, Action_Param.Max_Diff);
    bool D2z = Abs_Diff_Judge(leg[2].Foot.p.x, Action_Param.stand_height[2], Action_Param.Max_Diff);
    bool D3x = Abs_Diff_Judge(leg[3].Foot.p.x, 0, Action_Param.Max_Diff);
    bool D3z = Abs_Diff_Judge(leg[3].Foot.p.x, Action_Param.stand_height[3], Action_Param.Max_Diff);
    if ((D0x == true) && (D0z == true) && (D1x == true) && (D1z == true) && (D2x == true) && (D2z == true) && (D3x == true) && (D3z == true))
    {
        state = true;
    }
    else
    {
        state = false;
    }
    return state;
}

////void Cross_Leg(uint8_t legid, float up, float s, float down, float t) // ����
////{

////    float up_t = 1;     // ��ֱ�����˶�ʱ��(s)
////    float s_t = 1;      // ˮƽ�˶�ʱ��(s)
////    float down_t = 0.7; // ��ֱ�����˶�ʱ��(s)
////    static Point target_p[4];
////    if (t == 0) // ͨ��Get_Target_Position()�����õ�����˶���Ŀ��λ��
////    {
////        for (uint8_t i = 0; i < 4; i++)
////        {
////            target_p[i] = Get_Target_Position(i);
////        }
////    }
////    int8_t direction = 1;
////    if (legid == 2 || legid == 3)
////    {
////        direction = -1;
////    }

////    if (t <= up_t) // ��ֱ�����˶�
////    {
////        line(legid, direction * target_p[legid].x, target_p[legid].y,
////             direction * target_p[legid].x, target_p[legid].y - up, up_t, t);
////    }
////    else if ((t > up_t) && (t <= (up_t + s_t))) // ˮƽ�˶�
////    {
////        line(legid, direction * target_p[legid].x, target_p[legid].y - up,
////             (direction * target_p[legid].x + s), target_p[legid].y - up, s_t, t - up_t);
////    }
////    else if ((t > up_t + s_t) && (t <= (up_t + s_t + down_t))) // ��ֱ�����˶�
////    {
////        line(legid, (direction * target_p[legid].x + s), target_p[legid].y - up,
////             (direction * target_p[legid].x + s), target_p[legid].y - up + down, down_t, t - up_t - s_t);
////    }
////    else if (t > (up_t + s_t + down_t))
////    {
////        SetDogChangeAble(ENABLE);
////    }
////}

/* ---------------------------------------״̬��----------------------------------------- */

/* ---------------------------------------����------------------------------------------ */
void start_enter(void)
{
}
void start_run(void)
{
}

void start_exit(void)
{
}

/* ---------------------------------------����------------------------------------------ */
void keep_enter()
{
    Action_Param.Launch_Flag = Launch;
    SetDogChangeAble(DISABLE); // �������ǰ��ֹ�л�״̬
    GetTick();
    Change_Dog_Pitch_State(Follow_Ground);
    tick_start = Time_Points_Param.Now_Tick; // ��ȡ��ǰʱ��
    Set_All_Motor_Param(&Action_Param.Posctrl_keep);
}
extern float data2send[10];

void keep_run()
{
    GetTick();
    float t = Time_Points_Param.Now_Tick - tick_start;
		if(Action_Param.keep_type == 0)
		{
			bool state = Judge_Keep_State();
			if (t > (Action_Param.stand_tm / 2.0f)) // t>����Ķ��������Stand_Up�����������һ��
					state = true;
			if (state == false)
			{
					Stand_Up(t, Action_Param.stand_tm / 2.0f, Action_Param.stand_height); // t>����Ķ��������Stand_Up�����������һ��
			}
			else
			{
					Keep_All_Leg(0, 0, 0, 0,
											 Action_Param.stand_height[0], Action_Param.stand_height[1],
											 Action_Param.stand_height[2], Action_Param.stand_height[3], &Action_Param.VMC_Stance_Keep[0]);
					SetDogChangeAble(ENABLE);
			}
		}
		else if(Action_Param.keep_type == 1)
		{
			for (int i = 0; i <= 3; i++)
			{
					Get_Now_Pos_Spd(i);
					Coordinate_Transform(&leg[i].Foot.p, &Action_Param.Init_Pos[i], -Euler_Angle.Pitch, &leg[i], i);
			}
			Keep_All_Leg(Action_Param.Init_Pos[0].x,  Action_Param.Init_Pos[1].x, 
									-Action_Param.Init_Pos[2].x, -Action_Param.Init_Pos[3].x,
									 Action_Param.stand_height[0], Action_Param.stand_height[1],
									 Action_Param.stand_height[2], Action_Param.stand_height[3], &Action_Param.VMC_Stance_Keep[0]);
			SetDogChangeAble(ENABLE);
		}
}

/* ---------------------------------------�¶�----------------------------------------- */
void squat_enter()
{
    Action_Param.Launch_Flag = Launch;
    SetDogChangeAble(DISABLE); // �������ǰ��ֹ�л�״̬
    GetTick();
    Change_Dog_Pitch_State(Follow_Ground);
    tick_start = Time_Points_Param.Now_Tick; // ��ȡ��ǰʱ��
    Set_All_Motor_Param(&Action_Param.Posctrl_Squat);
}

void squat_run() // ִ�ж���
{
    GetTick();
    if (leg[0].ctrl_state == Pos_Ctrl)
    {
        Change_Ctrl_State(Force_Ctrl);
    }
    float t = Time_Points_Param.Now_Tick - tick_start;
    if (t <= Action_Param.squat_tm)
        Squat_Down(t, Action_Param.squat_tm, Action_Param.squat_x, Action_Param.squat_z);
    if (t > Action_Param.squat_tm)
    {
        Keep_All_Leg(Action_Param.squat_x[0], Action_Param.squat_x[1],
                     Action_Param.squat_x[2], Action_Param.squat_x[3],
                     Action_Param.squat_z[0], Action_Param.squat_z[1],
                     Action_Param.squat_z[2], Action_Param.squat_z[3],
                     &Action_Param.VMC_Stance_Keep[0]); // ��λ��
        SetDogChangeAble(ENABLE);
    }
}

/* ---------------------------------------վ��------------------------------------------ */

void stand_enter(void) // �л���stand״̬ʱִ��һ��
{
    if (leg[0].ctrl_state == Pos_Ctrl)
    {
        Change_Ctrl_State(Force_Ctrl);
    }

    Action_Param.Launch_Flag = Launch;
    SetDogChangeAble(DISABLE); // �������ǰ��ֹ�л�״̬
    GetTick();
    tick_start = Time_Points_Param.Now_Tick; // ��ȡ��ǰʱ��
    Set_All_Motor_Param(&Action_Param.Posctrl_stand);
    Change_Dog_Pitch_State(Follow_Ground);
}
void stand_run(void) // ִ�ж���
{
    GetTick();
    float t = Time_Points_Param.Now_Tick - tick_start;
    Stand_Up(t, Action_Param.stand_tm, Action_Param.stand_height);
}

void stand_exit(void)
{
}

/* ---------------------------------------����------------------------------------------ */
void crossleg_enter()
{
    if (leg[0].ctrl_state == Pos_Ctrl)
    {
        Change_Ctrl_State(Force_Ctrl);
    }

    Action_Param.Launch_Flag = Launch;
    SetDogChangeAble(DISABLE); // �������ǰ��ֹ�л�״̬
    GetTick();
    tick_start = Time_Points_Param.Now_Tick; // ��ȡ��ǰʱ��
    Change_Dog_Pitch_State(Follow_Ground);
}

void crossleg_run()
{
    GetTick();
    float t = Time_Points_Param.Now_Tick - tick_start;
    Cross_leg(t);
}

////////void test1_enter()
////////{
////////}

////////void test1_run()
////////{
////////}

////////void test2_enter()
////////{
////////}

////////void test2_run()
////////{
////////}

////////void test3_enter()
////////{
////////}

////////void test3_run()
////////{
////////}
