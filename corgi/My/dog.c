#include "dog.h"
#include "math.h"
#include "stm32h7xx_hal.h"
#include "timer.h"
#include "A1_motor.h"
#include "fsm.h"
#include "string.h"
#include "imu.h"
#include "handkey.h"
#include "walk.h"
#include "jump.h"
#include "ANO_TC.h"
#include "vision.h"
#include "action.h"
#include "delay_DWT.h"
#include "pos_ctrl.h"
#include "speed_race.h"

// ��ʼλ����ָ2pi/9�еľ���λ��,�����һȦ�оŸ�����λ��,���ݳ�ʼ�ڷ�λ�ò�ͬ,�������ת����ľ���λ����ȥ,������֤ÿ���ϵ�ʱ�����ʼ�Ƕ�һ��
// �߶���С10cm
// id        //motor
// 10  10    10   76

// 10  10    32   54

// �ȳ����0.14667m �0.4048m

LEG leg[4];
Euler Euler_Angle;
extern A1_Motor_Struct motor[8]; // ���
extern float data2send[10];
/* ------------------------------------------�˶�ѧ����------------------------------------------------ */

/*

�ⲿ��������ת���������ֱ��ǣ�
    �ؽ�->��ˣ�Kinematic_Solution
    ���->�ؽڣ�Kinematic_Inversion
    ���->��ˣ�Motor_Convert_To_Foot
    ���->�����Foot_Convert_To_Motor
    ���->�ؽڣ�Motor_Convert_To_Joint
    �ؽ�->�����Joint_Convert_To_Motor
    �ؽں��������ϵ��ת�������˶�ѧ�������,����ע��ؽڵ��������͵��ʵ�ʱ������ϵ�����һЩ����,������������ע��
*/

/**
 * @brief �������˶�ѧ����,������ת��,������λ�ã�������ת��,�������ٶ�
 * @param param �����Ƚṹ�����һЩ�����Ķ���
 * @param Joint ������ڹؽ�����ϵ���ٶ� λ��
 * @param Foot �����������ٶ� λ�ýṹ��
 */
void Kinematic_Solution(Param *param, foot *Foot, Kine *Joint) // ��ʵ����Motor_Covert_To_Foot,�ؽ�����ϵ���������ϵ
{
    float xb, xd, zb, zd;
    float A, B, C;
    float vbx, vbz, vdx, vdz;
    float sin_Phi1, sin_Phi2, cos_Phi1, cos_Phi2, w_Phi1;

    // ���ݵ��ת�Ǽ������λ��
    xb = param->rod[0] * cosf(Joint->Pos.Alpha);
    xd = param->rod[2] * cosf(Joint->Pos.Beta);
    zb = param->rod[0] * sinf(Joint->Pos.Alpha);
    zd = param->rod[2] * sinf(Joint->Pos.Beta);

    A = 2 * param->rod[1] * (xb + xd);
    B = 2 * param->rod[1] * (zb + zd);
    C = param->rod[1] * param->rod[1] + (xb + xd) * (xb + xd) + (zb + zd) * (zb + zd) - param->rod[3] * param->rod[3];

    cos_Phi1 = (-C * A - B * sqrtf(A * A - B * B - C * C)) / (A * A + B * B);
    cos_Phi2 = (xb - xd + param->rod[1] * cos_Phi1) / param->rod[3];

    sin_Phi1 = (-C - A * cos_Phi1) / B;
    sin_Phi2 = (zb + zd + param->rod[1] * sin_Phi1) / param->rod[3];

    Foot->p.x = xb + param->rod[1] * cos_Phi1;
    Foot->p.z = zb + param->rod[1] * sin_Phi1;

    // ���ݵ�����ٶȼ�������ٶ�
    vbx = -param->rod[0] * Joint->Spd.Alpha * sinf(Joint->Pos.Alpha);
    vbz = param->rod[0] * Joint->Spd.Alpha * cosf(Joint->Pos.Alpha);
    vdx = -param->rod[2] * Joint->Spd.Beta * sinf(Joint->Pos.Beta);
    vdz = param->rod[2] * Joint->Spd.Beta * cosf(Joint->Pos.Beta);

    w_Phi1 = ((vbx - vdx) * cos_Phi2 + (vbz - vdz) * sin_Phi2) / (param->rod[1] * (sin_Phi1 * cos_Phi2 - sin_Phi2 * cos_Phi1));

    Foot->v.x = vbx - param->rod[1] * w_Phi1 * sin_Phi1;
    Foot->v.z = vbz + param->rod[1] * w_Phi1 * cos_Phi1;
}

void Kinematic_Inversion_Update(Param *param, foot *Foot, Kine *Joint)
{
    // �������λ�ü���ؽڵ��ת��
    float now_a1, a2;
    float AOB;
    float l;
    float Dleta_Angle1, Dleta_Angle2;

    now_a1 = atan2f(Foot->p.z, Foot->p.x);
    if (param->last_a1 == 0)
    {
        param->true_a1 += now_a1;
        param->last_a1 = now_a1;
    }
    if (now_a1 > (param->last_a1))
    {
        Dleta_Angle1 = now_a1 - param->last_a1;
        Dleta_Angle2 = -6.2832f + now_a1 - param->last_a1;
    }
    else
    {
        Dleta_Angle1 = now_a1 - param->last_a1;
        Dleta_Angle2 = 6.2832f + now_a1 - param->last_a1;
    }

    if ((fabs(Dleta_Angle1)) > (fabs(Dleta_Angle2)))
    {
        param->true_a1 += Dleta_Angle2;
    }
    else
    {
        param->true_a1 += Dleta_Angle1;
    }
    param->last_a1 = now_a1;

    l = sqrtf(Foot->p.z * Foot->p.z + Foot->p.x * Foot->p.x);
    a2 = acosf((param->rod[1] * param->rod[1] - l * l - param->rod[0] * param->rod[0]) / (2.0f * param->rod[1] * l));

    AOB = atan2f(param->rod[1] * sinf(a2), (l + param->rod[1] * cosf(a2)));
    Joint->Pos.Alpha = param->true_a1 + AOB;
    Joint->Pos.Beta = 2.0f * param->true_a1 - Joint->Pos.Alpha + 6.2832f;
}

/**
 * @brief �������˶�ѧ���,�������λ��,������ת�ǣ���������ٶ�,������ת��
 * @param param �����Ƚṹ�����һЩ�����Ķ���
 * @param Joint ������ڹؽ�����ϵ���ٶ� λ��
 * @param Foot �����������ٶ� λ�ýṹ��
 */
void Kinematic_Inversion(Param *param, foot *Foot, Kine *Joint)
{
    Kinematic_Inversion_Update(param, Foot, Joint);

    // ��������ٶȼ���ؽڵ�����ٶ�
    float sin_Phi1, sin_Phi2, cos_Phi1, cos_Phi2;
    float xb, xd, zb, zd;
    float A, B, C;

    xb = param->rod[0] * cosf(Joint->Pos.Alpha);
    xd = param->rod[2] * cosf(Joint->Pos.Beta);
    zb = param->rod[0] * sinf(Joint->Pos.Alpha);
    zd = param->rod[2] * sinf(Joint->Pos.Beta);

    A = 2 * param->rod[1] * (xb - xd);
    B = 2 * param->rod[1] * (zb - zd);
    C = param->rod[1] * param->rod[1] + (xb - xd) * (xb - xd) + (zb - zd) * (zb - zd) - param->rod[3] * param->rod[3];

    cos_Phi1 = (-C * A - B * sqrtf(A * A + B * B - C * C)) / (A * A + B * B);
    cos_Phi2 = (xb - xd + param->rod[1] * cos_Phi1) / param->rod[3];

    sin_Phi1 = (-C - A * cos_Phi1) / B;
    sin_Phi2 = (zb - zd + param->rod[1] * sin_Phi1) / param->rod[3];

    Joint->Spd.Alpha = (Foot->v.x * cos_Phi1 + Foot->v.z * sin_Phi1) / (param->rod[0] * (sin_Phi1 * cosf(Joint->Pos.Alpha) - sinf(Joint->Pos.Alpha) * cos_Phi1));
    Joint->Spd.Beta = (Foot->v.x * cos_Phi2 + Foot->v.z * sin_Phi2) / (param->rod[2] * (sin_Phi2 * cosf(Joint->Pos.Beta) - sinf(Joint->Pos.Beta) * cos_Phi2));
}

/**
 * @brief �������˶�ѧ���,�������λ��,������ת�ǣ���������ٶ�,������ת��
 * @param leg �����Ƚṹ���Ӧ�����пɱ����
 * @param param �����Ƚṹ�����һЩ�����Ķ���
 * @param VMC_Output ������ϵ��x,z�����������������С
 * @param Motor_Torque �ؽڵ���������
 * @warning ʹ���������ǰ������Get_Now_Pos_Spd������ùؽ�����ϵ�ĵ��λ��
 */
void VMC_Jacobi_Matrix(LEG *leg, Param *param, Point *VMC_Output, Order *Motor_Torque)
{
    // ���Phi1,Phi2,���˶�ѧ�����һģһ��
    float xb, xd, zb, zd;
    float A, B, C;
    float sin_Phi1, sin_Phi2, cos_Phi1, cos_Phi2;

    xb = param->rod[0] * cosf(leg->Joint.Pos.Alpha);
    xd = param->rod[2] * cosf(leg->Joint.Pos.Beta);
    zb = param->rod[0] * sinf(leg->Joint.Pos.Alpha);
    zd = param->rod[2] * sinf(leg->Joint.Pos.Beta);

    A = 2 * param->rod[1] * (xb - xd);
    B = 2 * param->rod[1] * (zb - zd);
    C = param->rod[1] * param->rod[1] + (xb - xd) * (xb - xd) + (zb - zd) * (zb - zd) - param->rod[3] * param->rod[3];

    cos_Phi1 = (-C * A - B * sqrtf(A * A + B * B - C * C)) / (A * A + B * B);
    cos_Phi2 = (xb - xd + param->rod[1] * cos_Phi1) / param->rod[3];

    sin_Phi1 = (-C - A * cos_Phi1) / B;
    sin_Phi2 = (zb - zd + param->rod[1] * sin_Phi1) / param->rod[3];
    float sin_Phi1_Phi2 = sin_Phi1 * cos_Phi2 - sin_Phi2 * cos_Phi1;

    // �����ſɱȾ���
    // [ Jacobi[0][0] Jacobi[0][1] ] [Fx]
    // [ Jacobi[1][0] Jacobi[1][1] ] [Fy]
    float Jacobi[2][2];
    Jacobi[0][0] = param->rod[0] * sin_Phi2 * (sinf(leg->Joint.Pos.Alpha) * cos_Phi1 - sin_Phi1 * cosf(leg->Joint.Pos.Alpha)) / sin_Phi1_Phi2;
    Jacobi[1][0] = param->rod[2] * sin_Phi1 * (sin_Phi2 * cosf(leg->Joint.Pos.Beta) - sinf(leg->Joint.Pos.Beta) * cos_Phi1) / sin_Phi1_Phi2;
    Jacobi[0][1] = -param->rod[0] * cos_Phi2 * (sinf(leg->Joint.Pos.Alpha) * cos_Phi1 - sin_Phi1 * cosf(leg->Joint.Pos.Alpha)) / sin_Phi1_Phi2;
    Jacobi[1][1] = -param->rod[2] * cos_Phi1 * (sin_Phi2 * cosf(leg->Joint.Pos.Beta) - sinf(leg->Joint.Pos.Beta) * cos_Phi1) / sin_Phi1_Phi2;

    Motor_Torque->Alpha = (VMC_Output->x * Jacobi[0][0] + VMC_Output->z * Jacobi[0][1]) * param->motor_Alpha_dir;
    Motor_Torque->Beta = (VMC_Output->x * Jacobi[1][0] + VMC_Output->z * Jacobi[1][1]) * param->motor_Beta_dir;
}

/**
 * @brief  ��Ϊ�������û�о��Ա�����,�ϵ�ʱ��������ֵ����0������Ҫ��һ���������ϵ�͹ؽ�����ϵ��ת��
           �������ϵ�½Ƕ� = �����ת����*����Ѿ���ת�ĽǶ�+�������ϵ�³�ʼ�Ƕ�
           �����ת�����泯���,��ʱ����ת��Ϊ+,˳ʱ���Ϊ-
           ����Ѿ���ת�ĽǶ�=�ؽ�����ϵ�µ����ǰ�Ƕ�-�ؽ�����ϵ�µ����ʼ�Ƕ�
           �������ϵ�³�ʼ�Ƕȣ����Ȱڵ���ʼλ��(�ȸպÿ�ס�����̶���),����ϵ�,��ʱ����Ļش�ֵ
 * @param  param �����Ƚṹ�����һЩ�����Ķ���
 * @param  Joint ������ڹؽ�����ϵ���ٶ� λ��
 * @param  Motor ������ڵ������ϵ���ٶ� λ��
 */
void Joint_Convert_To_Motor(Param *param, Kine *Joint, Kine *Motor) // �ؽ�����ϵ�½Ƕ�����ٶ�ת�����������ϵ�½Ƕ�����ٶ�
{
    Motor->Pos.Alpha = param->motor_Alpha_dir * (Joint->Pos.Alpha - param->start_Alpha) + param->start_motor_Alpha;
    Motor->Pos.Beta = param->motor_Beta_dir * (Joint->Pos.Beta - param->start_Beta) + param->start_motor_Beta;
    Motor->Spd.Alpha = param->motor_Alpha_dir * Joint->Spd.Alpha;
    Motor->Spd.Beta = param->motor_Beta_dir * Joint->Spd.Beta;
}

void Motor_Convert_To_Joint(Param *param, Kine *Joint, Kine *Motor) // �������ϵ�½Ƕ�����ٶ�ת�����ؽ�����ϵ�½Ƕ�����ٶ�
{
    Joint->Pos.Alpha = param->motor_Alpha_dir * (Motor->Pos.Alpha - param->start_motor_Alpha) + param->start_Alpha;
    Joint->Pos.Beta = param->motor_Beta_dir * (Motor->Pos.Beta - param->start_motor_Beta) + param->start_Beta;
    Joint->Spd.Alpha = param->motor_Alpha_dir * Motor->Spd.Alpha;
    Joint->Spd.Beta = param->motor_Beta_dir * Motor->Spd.Beta;
}

void Foot_Convert_To_Motor(Param *param, foot *Foot, Kine *Joint, Kine *Motor) // �������ϵ�½Ƕ�����ٶ�ת�����������ϵ�½Ƕ�����ٶ�
{
    Kinematic_Inversion(param, Foot, Joint);
    Joint_Convert_To_Motor(param, Joint, Motor);
}

void Motor_Convert_To_Foot(Param *param, foot *Foot, Kine *Joint, Kine *Motor) // �������ϵ�½Ƕ�����ٶ�ת�����������ϵ�½Ƕ�����ٶ�
{
    Motor_Convert_To_Joint(param, Joint, Motor);
    Kinematic_Solution(param, Foot, Joint);
}

/* ------------------------------------------ͨѶ����------------------------------------------------ */

/**
 * @brief  ���ط���Ŀ������
 * @param  leg �ȴ�ṹ��
 */
void Set_Motor_Target_Torque(LEG *leg)
{
    if (leg->Motor_Torque.Alpha >= 33.5f)
        leg->Motor_Torque.Alpha = 33.5f;

    if (leg->Motor_Torque.Alpha <= -33.5f)
        leg->Motor_Torque.Alpha = -33.5f;

    if (leg->Motor_Torque.Beta >= 33.5f)
        leg->Motor_Torque.Beta = 33.5f;

    if (leg->Motor_Torque.Beta <= -33.5f)
        leg->Motor_Torque.Beta = -33.5f;

    motor[leg->const_param.motor_Alpha_id + 2 * leg->const_param.id].target.T = leg->Motor_Torque.Alpha;
    motor[leg->const_param.motor_Beta_id + 2 * leg->const_param.id].target.T = leg->Motor_Torque.Beta;

    motor[leg->const_param.motor_Alpha_id + 2 * leg->const_param.id].target.Pos = motor[leg->const_param.motor_Alpha_id + 2 * leg->const_param.id].measure.Pos;
    motor[leg->const_param.motor_Alpha_id + 2 * leg->const_param.id].target.W = motor[leg->const_param.motor_Alpha_id + 2 * leg->const_param.id].measure.W;

    motor[leg->const_param.motor_Beta_id + 2 * leg->const_param.id].target.Pos = motor[leg->const_param.motor_Beta_id + 2 * leg->const_param.id].measure.Pos;
    motor[leg->const_param.motor_Beta_id + 2 * leg->const_param.id].target.W = motor[leg->const_param.motor_Beta_id + 2 * leg->const_param.id].measure.W;
}

/**
 * @brief  λ�ط���Ŀ���ٶ�λ��
 * @param  leg �ȴ�ṹ��
 */
void Set_Motor_Target_Pos(LEG *leg)
{
    motor[leg->const_param.motor_Alpha_id + 2 * leg->const_param.id].target.T = 0;
    motor[leg->const_param.motor_Beta_id + 2 * leg->const_param.id].target.T = 0;

    motor[leg->const_param.motor_Alpha_id + 2 * leg->const_param.id].target.Pos = leg->Pctrl.Motor_Target.Pos.Alpha;
    motor[leg->const_param.motor_Alpha_id + 2 * leg->const_param.id].target.W = leg->Pctrl.Motor_Target.Spd.Alpha;

    motor[leg->const_param.motor_Beta_id + 2 * leg->const_param.id].target.Pos = leg->Pctrl.Motor_Target.Pos.Beta;
    motor[leg->const_param.motor_Beta_id + 2 * leg->const_param.id].target.W = leg->Pctrl.Motor_Target.Spd.Beta;
}

/* ------------------------------------------����ĳ�ʼ��------------------------------------------------ */

void Leg_Initpos_Reset(void)
{
    if ((motor[0].mode == 0) && (motor[1].mode == 0) && (motor[2].mode == 0) && (motor[3].mode == 0) && (motor[4].mode == 0) && (motor[5].mode == 0) && (motor[6].mode == 0) && (motor[7].mode == 0))
    {
        leg[0].const_param.start_motor_Alpha = motor[1].measure.Pos;
        leg[0].const_param.start_motor_Beta = motor[0].measure.Pos;

        leg[1].const_param.start_motor_Alpha = motor[3].measure.Pos;
        leg[1].const_param.start_motor_Beta = motor[2].measure.Pos;

        leg[2].const_param.start_motor_Alpha = motor[5].measure.Pos;
        leg[2].const_param.start_motor_Beta = motor[4].measure.Pos;

        leg[3].const_param.start_motor_Alpha = motor[7].measure.Pos;
        leg[3].const_param.start_motor_Beta = motor[6].measure.Pos;
    }
}

void leg_init() // �������ֻ��Ĵ˲���
{
    uint8_t id = 0;
    // ��0
    id = 0;
    leg[id].const_param.motor_Alpha_id = 1;
    leg[id].const_param.motor_Alpha_dir = 1;
    leg[id].const_param.start_Alpha = -116.41f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Alpha = 0.3352;

    leg[id].const_param.motor_Beta_id = 0;
    leg[id].const_param.motor_Beta_dir = -1;
    leg[id].const_param.start_Beta = -203.01f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Beta = 0.3107;

    // ��1
    id = 1;
    leg[id].const_param.motor_Alpha_id = 1;
    leg[id].const_param.motor_Alpha_dir = 1;
    leg[id].const_param.start_Alpha = -116.41f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Alpha = 0.1255;

    leg[id].const_param.motor_Beta_id = 0;
    leg[id].const_param.motor_Beta_dir = -1;
    leg[id].const_param.start_Beta = -203.01f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Beta = 0.1488;

    // ��2
    id = 2;
    leg[id].const_param.motor_Alpha_id = 1;
    leg[id].const_param.motor_Alpha_dir = 1;
    leg[id].const_param.start_Alpha = 23.01f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Alpha = 0.5963;

    leg[id].const_param.motor_Beta_id = 0;
    leg[id].const_param.motor_Beta_dir = -1;
    leg[id].const_param.start_Beta = -63.59f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Beta = 0.0851;

    // ��3
    id = 3;
    leg[id].const_param.motor_Alpha_id = 1;
    leg[id].const_param.motor_Alpha_dir = 1;
    leg[id].const_param.start_Alpha = 23.01f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Alpha = 0.2546;

    leg[id].const_param.motor_Beta_id = 0;
    leg[id].const_param.motor_Beta_dir = -1;
    leg[id].const_param.start_Beta = -63.59f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Beta = 0.0902;

    for (int i = 0; i < 4; i++)
    {
        leg[i].const_param.id = i;

        // �������˳���
        leg[i].const_param.rod[0] = 0.175f;
        leg[i].const_param.rod[1] = 0.3f;
        leg[i].const_param.rod[2] = 0.175f;
        leg[i].const_param.rod[3] = 0.3f;

        // ��ʼ���������
        leg[i].Motor.Pos.Alpha = motor[leg[i].const_param.motor_Alpha_id + 2 * i].measure.Pos;
        leg[i].Motor.Pos.Beta = motor[leg[i].const_param.motor_Beta_id + 2 * i].measure.Pos;

        // ������˳�ʼλ��
        Get_Now_Pos_Spd(i);
    }
}

/* ------------------------------------------��������------------------------------------------------ */

// ������ϵ����0Ϊ��׼,��xΪ��ʱ,��2��3����ϵ��xΪ��

/*

���ڴ�����ϵ�е�ʱ�䰲�ţ�
(1)1ms��һ�ζ�ʱ���ж�->tick+1,Ҳ����˵tick��¼���Դ򿪶�ʱ����ʼ�������е�ʱ��,����Ϊ���˶��Ľ���һֱ����¼��
(2)FSM�Ĳ��裺ִ��һ�ν��뺯��->ִ��n��ִ�к���,ֱ�����ִ�ж���->ִ��һ���˳�����
(3)Ҳ����˵���ڽ��뺯���л�ÿ�ʼִ�ж���ʱ��ϵͳʱ��tick_start,Ȼ��ʼִ��stand_run����,
���Ų��ϵ�ִ��,GetTick()�����ķ���ֵ��������,�Ұ�ִ�ж����е�ϵͳʱ��͸տ�ʼִ�ж�����ϵͳʱ������,
��¼����t��������,��ôt����һ��һ��ϵͳִ��ʱ�����ɢ�㡣�����趨stand����ִ��2s,
��ôt�ļ�������0,1,2,3,4,5,......,2000,�Ӷ������˹���ʱ����ȷֲ��ĵ�,�������ɹ켣
*/

void Get_Now_Pos_Spd(uint8_t legid)
{
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
    {
        switch (legid)
        {
        case 0:
            leg[0].Motor.Pos.Alpha = motor[1].measure.Pos;
            leg[0].Motor.Pos.Beta = motor[0].measure.Pos;
            leg[0].Motor.Spd.Alpha = motor[1].measure.W;
            leg[0].Motor.Spd.Beta = motor[0].measure.W;

            // �������ϵת�������,�������λ��
            Motor_Convert_To_Joint(&leg[0].const_param, &leg[0].Joint, &leg[0].Motor);
            Kinematic_Solution(&leg[0].const_param, &leg[0].Foot, &leg[0].Joint);

            // �����˶�ѧ�������ĵ��ת��
            if (Get_Now_Ctrl_State() == Force_Ctrl)
                Kinematic_Inversion_Update(&leg[0].const_param, &leg[0].Foot, &leg[0].Joint);

            break;

        case 1:
            leg[1].Motor.Pos.Alpha = motor[3].measure.Pos;
            leg[1].Motor.Pos.Beta = motor[2].measure.Pos;
            leg[1].Motor.Spd.Alpha = motor[3].measure.W;
            leg[1].Motor.Spd.Beta = motor[2].measure.W;

            // �������ϵת�������,�������λ��
            Motor_Convert_To_Joint(&leg[1].const_param, &leg[1].Joint, &leg[1].Motor);
            Kinematic_Solution(&leg[1].const_param, &leg[1].Foot, &leg[1].Joint);

            // �����˶�ѧ�������ĵ��ת��
            if (Get_Now_Ctrl_State() == Force_Ctrl)
                Kinematic_Inversion_Update(&leg[1].const_param, &leg[1].Foot, &leg[1].Joint);

            break;

        case 2:
            leg[2].Motor.Pos.Alpha = motor[5].measure.Pos;
            leg[2].Motor.Pos.Beta = motor[4].measure.Pos;
            leg[2].Motor.Spd.Alpha = motor[5].measure.W;
            leg[2].Motor.Spd.Beta = motor[4].measure.W;

            // �������ϵת�������,�������λ��
            Motor_Convert_To_Joint(&leg[2].const_param, &leg[2].Joint, &leg[2].Motor);
            Kinematic_Solution(&leg[2].const_param, &leg[2].Foot, &leg[2].Joint);

            // �����˶�ѧ�������ĵ��ת��
            if (Get_Now_Ctrl_State() == Force_Ctrl)
                Kinematic_Inversion_Update(&leg[2].const_param, &leg[2].Foot, &leg[2].Joint);

            break;

        case 3:
            leg[3].Motor.Pos.Alpha = motor[7].measure.Pos;
            leg[3].Motor.Pos.Beta = motor[6].measure.Pos;
            leg[3].Motor.Spd.Alpha = motor[7].measure.W;
            leg[3].Motor.Spd.Beta = motor[6].measure.W;

            // �������ϵת�������,�������λ��
            Motor_Convert_To_Joint(&leg[3].const_param, &leg[3].Joint, &leg[3].Motor);
            Kinematic_Solution(&leg[3].const_param, &leg[3].Foot, &leg[3].Joint);

            // �����˶�ѧ�������ĵ��ת��
            if (Get_Now_Ctrl_State() == Force_Ctrl)
                Kinematic_Inversion_Update(&leg[3].const_param, &leg[3].Foot, &leg[3].Joint);

            break;
        }
    }
}

/**
 * @brief ��ת����任
 * @param Input
 * @param Output
 * @param Pitch
 */
void Coordinate_Transform(Point *Input, Point *Output, float Pitch, LEG *leg, uint8_t legid)
{
    if (leg->pitch_ctrl_state == Follow_Ground)
    {
        Output->x = Input->x * cosf(Pitch * 0.1217f) + Input->z * sinf(Pitch * 0.1217f);
        Output->z = -Input->x * sinf(Pitch * 0.1217f) + Input->z * cosf(Pitch * 0.1217f);
    }
    else
    {
        Output->x = Input->x;
        Output->z = Input->z;
    }
}

/**
 * @brief PD��������������
 * @param Param PD����
 * @param Target_Pos �����������ϵ����x,z�����ϵ�Ŀ������,��ֱ��/�����������õ�
 * @param Now_Pos �����������ϵ����x,z�����ϵĵ�ǰ����,�ɵ������ֵ+�˶�ѧ����õ�
 * @param VMC_Output ���ڵ���֧����,��������ϵ��x,z������Ҫ�������������С
 */
void PD_Control(PD *Param, Point *Target_Pos, Point *Now_Pos, Point *VMC_Output)
{
    // �������error
    Param->err_x = Target_Pos->x - Now_Pos->x;
    Param->err_z = Target_Pos->z - Now_Pos->z;

    // ���㲻��ȫD��
    Param->differ_x = Param->Kd_x * (1 - Param->k_x) * (Param->err_x - Param->last_err_x) + Param->k_x * Param->last_differ_x;
    Param->differ_z = Param->Kd_z * (1 - Param->k_z) * (Param->err_z - Param->last_err_z) + Param->k_z * Param->last_differ_z;

    // �������������
    VMC_Output->x = Param->Kp_x * Param->err_x + Param->differ_x + Param->Feedforward_x;
    VMC_Output->z = Param->Kp_z * Param->err_z + Param->differ_z + Param->Feedforward_z;

    // ���ݲ���
    Param->last_err_x = Param->err_x;
    Param->last_differ_x = Param->differ_x;
    Param->last_err_z = Param->err_z;
    Param->last_differ_z = Param->differ_z;
}

/**
 * @brief �����һ��ֱ��
 * @param legid �����ȵ�id,�ж�����ֻ����ֱ��
 * @param start_x,start_z ֱ��ʼ��
 * @param end_x,end_z ֱ��ĩ��
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @param Swing PD���ưڶ������
 */
void line(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float T, float t, PD *Swing)
{
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
    {
        Get_Now_Pos_Spd(legid);

        Point p = {0, 0, 0};
        p = line_Generator(legid, start_x, start_z, end_x, end_z, T, t);
        Coordinate_Transform(&p, &leg[legid].Foot_Target.p, Euler_Angle.Pitch, &leg[legid], legid);

        // leg[legid].Target_p.x = VMC_Filter(leg[legid].Target_p.x,leg[legid].now.Pos_Filter.x,leg[legid].now.Pos_Filter.Max_AMP_x);
        // leg[legid].Target_p.z = VMC_Filter(leg[legid].Target_p.z,leg[legid].now.Pos_Filter.z,leg[legid].now.Pos_Filter.Max_AMP_z);

        PD_Control(Swing, &leg[legid].Foot_Target.p, &leg[legid].Foot.p, &leg[legid].VMC_Output_Force);
        VMC_Jacobi_Matrix(&leg[legid], &leg[legid].const_param,
                          &leg[legid].VMC_Output_Force, &leg[legid].Motor_Torque);

        // leg[legid].VMC_Output_Force.x = VMC_Filter(leg[legid].VMC_Output_Force.x, leg[legid].now.Force_Filter.x, leg[legid].now.Force_Filter.Max_AMP_x);
        // leg[legid].VMC_Output_Force.z = VMC_Filter(leg[legid].VMC_Output_Force.z, leg[legid].now.Force_Filter.z, leg[legid].now.Force_Filter.Max_AMP_z);
        Set_Motor_Target_Torque(&leg[legid]);
    }
}

/**
 * @brief �����һ������
 * @param legid �����ȵ�id,�ж�����ֻ����ֱ��
 * @param start_x,start_z ����ʼ��
 * @param end_x ����ĩ��
 * @param max_z ���߶���z����
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @param Swing PD���ưڶ������
 */
void Cycloid(uint8_t legid, float start_x, float start_z, float end_x, float max_z, float T, float t, PD *Swing)
{
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
    {
        Get_Now_Pos_Spd(legid);

        Point p = {0, 0, 0};
        p = Cycloid_Generator(legid, start_x, start_z, (end_x - start_x), (start_z - max_z), T, t);
        Coordinate_Transform(&p, &leg[legid].Foot_Target.p, Euler_Angle.Pitch, &leg[legid], legid);

        // leg[legid].Target_p.x = VMC_Filter(leg[legid].Target_p.x,leg[legid].now.Pos_Filter.x,leg[legid].now.Pos_Filter.Max_AMP_x);
        // leg[legid].Target_p.z = VMC_Filter(leg[legid].Target_p.z,leg[legid].now.Pos_Filter.z,leg[legid].now.Pos_Filter.Max_AMP_z);

        PD_Control(Swing, &leg[legid].Foot_Target.p, &leg[legid].Foot.p, &leg[legid].VMC_Output_Force);
        VMC_Jacobi_Matrix(&leg[legid], &leg[legid].const_param,
                          &leg[legid].VMC_Output_Force, &leg[legid].Motor_Torque);

        // leg[legid].VMC_Output_Force.x = VMC_Filter(leg[legid].VMC_Output_Force.x, leg[legid].now.Force_Filter.x, leg[legid].now.Force_Filter.Max_AMP_x);
        // leg[legid].VMC_Output_Force.z = VMC_Filter(leg[legid].VMC_Output_Force.z, leg[legid].now.Force_Filter.z, leg[legid].now.Force_Filter.Max_AMP_z);

        Set_Motor_Target_Torque(&leg[legid]);
    }
}

/*
����˵һ��bezier����ϵ�Ķ���:��(start_x,start_z)Ϊԭ��,�����������Ϊx������,��ֱ����Ϊz������
*/

/**
 * @brief ����߱���������,�ڶ���ʹ��
 * @param legid �����ȵ�id,�ж�����ֻ���߰���(�漰�ٶ�ȡ��ȡ��)
 * @param start_x,start_z ������ϵ������ʼ��
 * @param end_x,end_z ������ϵ������ĩ��
 * @param max_z ���ߵ����߶�,���������д0,��������ϵz�������Ϊ-,��������ϵz�Ḻ���Ϊ+
 * @param Delta_x,Delta_z ��������ϵ��,���������ߵ�ԭ������
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @param Swing PD���ưڶ������
 */
void Bezier(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t, PD *Swing)
{
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
    {
        Get_Now_Pos_Spd(legid);

        Point p = {0, 0, 0};
        p = Bezier_Generator(legid, start_x, start_z, end_x, end_z, max_z, Delta_x, Delta_z, T, t);
        Coordinate_Transform(&p, &leg[legid].Foot_Target.p, Euler_Angle.Pitch, &leg[legid], legid);

        // leg[legid].Target_p.x = VMC_Filter(leg[legid].Target_p.x,leg[legid].now.Pos_Filter.x,leg[legid].now.Pos_Filter.Max_AMP_x);
        // leg[legid].Target_p.z = VMC_Filter(leg[legid].Target_p.z,leg[legid].now.Pos_Filter.z,leg[legid].now.Pos_Filter.Max_AMP_z);

        PD_Control(Swing, &leg[legid].Foot_Target.p, &leg[legid].Foot.p, &leg[legid].VMC_Output_Force);
        VMC_Jacobi_Matrix(&leg[legid], &leg[legid].const_param,
                          &leg[legid].VMC_Output_Force, &leg[legid].Motor_Torque);

        // leg[legid].VMC_Output_Force.x = VMC_Filter(leg[legid].VMC_Output_Force.x, leg[legid].now.Force_Filter.x, leg[legid].now.Force_Filter.Max_AMP_x);
        // leg[legid].VMC_Output_Force.z = VMC_Filter(leg[legid].VMC_Output_Force.z, leg[legid].now.Force_Filter.z, leg[legid].now.Force_Filter.Max_AMP_z);

        Set_Motor_Target_Torque(&leg[legid]);
    }
}

/**
 * @brief ����߱���������,walk״̬֧����ʹ��
 * @param legid �����ȵ�id,�ж�����ֻ���߰���(�漰�ٶ�ȡ��ȡ��)
 * @param start_x,start_z ������ϵ������ʼ��
 * @param end_x,end_z ������ϵ������ĩ��
 * @param max_z ���ߵ����߶�,���������д0,��������ϵz�������Ϊ-,��������ϵz�Ḻ���Ϊ+
 * @param Delta_x,Delta_z ��������ϵ��,���������ߵ�ԭ������
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @param Stance ֧����֧�ŵĲ���
 */
void Bezier_Stance(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t, PD *Stance)
{
    if (legid <= 3 && legid >= 0) // �Ϸ�����Խ��
    {
        Get_Now_Pos_Spd(legid);

        Point p = {0, 0, 0};
        p = Bezier_Generator(legid, start_x, start_z, end_x, end_z, max_z, Delta_x, Delta_z, T, t);
        Coordinate_Transform(&p, &leg[legid].Foot_Target.p, Euler_Angle.Pitch, &leg[legid], legid);

        // leg[legid].Target_p.x = VMC_Filter(leg[legid].Target_p.x,leg[legid].now.Pos_Filter.x,leg[legid].now.Pos_Filter.Max_AMP_x);
        // leg[legid].Target_p.z = VMC_Filter(leg[legid].Target_p.z,leg[legid].now.Pos_Filter.z,leg[legid].now.Pos_Filter.Max_AMP_z);

        PD_Control(Stance, &leg[legid].Foot_Target.p, &leg[legid].Foot.p, &leg[legid].VMC_Output_Force);
        VMC_Jacobi_Matrix(&leg[legid], &leg[legid].const_param,
                          &leg[legid].VMC_Output_Force, &leg[legid].Motor_Torque);

        // leg[legid].VMC_Output_Force.x = VMC_Filter(leg[legid].VMC_Output_Force.x, leg[legid].now.Force_Filter.x, leg[legid].now.Force_Filter.Max_AMP_x);
        // leg[legid].VMC_Output_Force.z = VMC_Filter(leg[legid].VMC_Output_Force.z, leg[legid].now.Force_Filter.z, leg[legid].now.Force_Filter.Max_AMP_z);

        Set_Motor_Target_Torque(&leg[legid]);
    }
}

/**
 * @brief ֱ��������
 * @param legid �����ȵ�id,�ж�����ֻ����ֱ��
 * @param start_x,start_z ֱ��ʼ��
 * @param end_x,end_z ֱ��ĩ��
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @return Target_Pos Ŀ��x,z
 */
Point line_Generator(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float T, float t)
{
    Point Target_Pos = {0, 0, 0}; // ��ʼ��Ŀ��xz
    float Now_Discrete_Time_Point = t;
    if (t > T)
        Now_Discrete_Time_Point = T;
    if (T != 0)
    {
        Target_Pos.x = (end_x - start_x) * (Now_Discrete_Time_Point / T) + start_x;
        Target_Pos.z = (end_z - start_z) * (Now_Discrete_Time_Point / T) + start_z;
    }
    else
    {
        Target_Pos.x = start_x;
        Target_Pos.z = start_z;
    }
    return Target_Pos;
}

/**
 * @brief ����������,-s��ʾ������
 * @param legid �����ȵ�id,�ж�����ֻ���߰���(�漰�ٶ�ȡ��ȡ��)
 * @param start_x,start_z ����ʼ��
 * @param s ������x���򳤶�
 * @param h ������z�������߶�
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @return Target_Pos Ŀ��x,z
 */
Point Cycloid_Generator(uint8_t legid, float start_x, float start_z, float s, float h, float T, float t)
{
    Point Target_Pos = {0, 0, 0}; // ��ʼ��Ŀ��xz
    float Now_Discrete_Time_Point = t;
    if (t > T)
        Now_Discrete_Time_Point = T;

    if (T != 0)
    {
        float fai = 2 * PI * Now_Discrete_Time_Point / T;
        Target_Pos.x = (s) * (fai - sinf(fai)) / (2 * PI) + start_x;
        Target_Pos.z = start_z - h * (1 - cosf(fai)) / 2.0f;
    }
    else
    {
        Target_Pos.x = start_x;
        Target_Pos.z = start_z;
    }

    return Target_Pos;
}

/**
 * @brief ����������������
 * @param legid �����ȵ�id
 * @param start_x,start_z ����ʼ��
 * @param end_x,end_z ����ĩ��
 * @param max_z ����������ϵ�����ߵ����߶�,���������д0
 * @param Delta_x,Delta_z ��������ϵ��,���������ߵ�ԭ������
 * @param T ������������Ҫ��ʱ��
 * @param t ��������Ѿ�����ʱ��
 * @return Target_Pos Ŀ��x,z
 */
Point Bezier_Generator(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t)
{
    // ��ʼ��
    // ������ϵ�µ�Ŀ��λ��,����յ�����
    Point Target_Pos_leg = {0, 0, 0};
    Point Start_leg = {0, 0, 0};
    Point End_leg = {0, 0, 0};
    // ����������ϵ�µ�Ŀ��λ��,����յ�����
    Point Target_Pos_Bezier = {0, 0, 0};
    Point Start_Bezier = {0, 0, 0};
    Point End_Bezier = {0, 0, 0};

    Start_leg.x = start_x;
    Start_leg.z = start_z;
    End_leg.x = end_x;
    End_leg.z = end_z;

    // ������ϵת������������������ϵ
    Bezier_Trans(legid, &Start_leg, &Start_Bezier, Delta_x, Delta_z);
    Bezier_Trans(legid, &End_leg, &End_Bezier, Delta_x, Delta_z);

    // ʱ�����
    float Now_Discrete_Time_Point = t;
    if (t > T)
        Now_Discrete_Time_Point = T;

    float Std_Time = 0;

    if (T != 0)
        Std_Time = Now_Discrete_Time_Point / T; // ��׼��ʱ��,��Χ0-1

    // ����Ŀ��x
    Target_Pos_Bezier.x = (1.0f - Std_Time) * (1.0f - Std_Time) * (1.0f - Std_Time) * Start_Bezier.x +
                          3.0f * (1.0f - Std_Time) * (1.0f - Std_Time) * Std_Time * Start_Bezier.x +
                          3.0f * (1.0f - Std_Time) * Std_Time * Std_Time * End_Bezier.x +
                          Std_Time * Std_Time * Std_Time * End_Bezier.x;

    Target_Pos_Bezier.z = (1 - Std_Time) * (1.0f - Std_Time) * (1.0f - Std_Time) * Start_Bezier.z +
                          3.0f * (1.0f - Std_Time) * (1 - Std_Time) * Std_Time * (max_z * 0.09f / 0.0675f) +
                          3.0f * (1.0f - Std_Time) * Std_Time * Std_Time * (max_z * 0.09f / 0.0675f) +
                          Std_Time * Std_Time * Std_Time * End_Bezier.z;

    // ��������������ϵת����������ϵ
    Bezier_Trans_Inv(legid, &Target_Pos_leg, &Target_Pos_Bezier, Delta_x, Delta_z);
    return Target_Pos_leg;
}

/**
 * @brief ������ϵת������������������ϵ
 * @param legid �����ȵ�id
 * @param Foot ����������ϵ�����λ������
 * @param Bezier �����������������ϵ�����λ������
 * @param Delta_x,Delta_z ��������ϵ��,���������ߵ�ԭ������
 */
void Bezier_Trans(uint8_t legid, Point *Foot, Point *Bezier, float Delta_x, float Delta_z)
{
    if (legid == 0 || legid == 1)
        Bezier->x = Foot->x - Delta_x;
    if (legid == 2 || legid == 3)
        Bezier->x = Delta_x - Foot->x;
    Bezier->z = Delta_z - Foot->z;
}
/**
 * @brief ��������������ϵת����������ϵ
 * @param legid �����ȵ�id
 * @param Foot ����������ϵ�����λ������
 * @param Bezier �����������������ϵ�����λ������
 * @param Delta_x,Delta_z ��������ϵ��,���������ߵ�ԭ������
 */
void Bezier_Trans_Inv(uint8_t legid, Point *Foot, Point *Bezier, float Delta_x, float Delta_z)
{
    if (legid == 0 || legid == 1)
        Foot->x = Delta_x + Bezier->x;
    if (legid == 2 || legid == 3)
        Foot->x = Delta_x - Bezier->x;
    Foot->z = Delta_z - Bezier->z;
}

#define FILTER_N 10
#define FIRST_LAG_PROPORTION 0.4

/**
 * @brief �޷�ƽ���˲���,��ȥ��Ƶ����(����),�ұ������ͻ��(��������+��������)
 * @param Input_Value �������ֵ
 * @param filter_buf �˲�����������ͷ��ַ
 * @param Max_AMP �������ͻ����
 * @return �������������ֵ
 */
float VMC_Filter(float Input_Value, float filter_buf[], float Max_AMP, float first_order_value, float first_order_last_value)
{
    first_order_last_value = Input_Value;
    return (1 - FIRST_LAG_PROPORTION) * first_order_value + FIRST_LAG_PROPORTION * Input_Value;
    /*
    int i;
    float filter_sum = 0;
    filter_buf[FILTER_N - 1] = Input_Value;
    if (((filter_buf[FILTER_N - 1] - filter_buf[FILTER_N - 2]) > Max_AMP) || ((filter_buf[FILTER_N - 2] - filter_buf[FILTER_N - 1]) > Max_AMP))
        filter_buf[FILTER_N - 1] = filter_buf[FILTER_N - 2];
    for (i = 0; i < FILTER_N - 1; i++)
    {
        filter_buf[i] = filter_buf[i + 1];
        filter_sum += filter_buf[i];
    }
    return (float)filter_sum / (FILTER_N - 1);*/
}

void Change_Dog_Pitch_State(Pitch_Ctrl_State state)
{
    leg[0].pitch_ctrl_state = state;
    leg[1].pitch_ctrl_state = state;
    leg[2].pitch_ctrl_state = state;
    leg[3].pitch_ctrl_state = state;
}

/**
 * @brief �ı丩����������ʽ
 * @param state
 * @param legid
 * @param Pitch �����������������ϵ�����λ������
 * @param Delta_x,Delta_z ��������ϵ��,���������ߵ�ԭ������
 */
void Change_Single_Pitch_State(Pitch_Ctrl_State state, uint8_t legid)
{
    leg[legid].pitch_ctrl_state = state;
}

float data2send[10];
extern action_param Action_Param;
extern float wait_for_jump;        // �ȴ������Ƿ�������ʱ��¼���м�ʱ��
extern uint8_t wait_for_jump_flag; // ��־λ,0��ʾ��δ��¼ʱ��,1��ʾ�Ѿ���¼ʱ��,����������δ��������,2��ʾ���Խ���ڶ�����Ծ
extern uint8_t memory_flag;
extern speed_race_param Auto_spd;
void ano_tc(void)
{
    data2send[0] = Euler_Angle.Yaw;
    data2send[1] = Euler_Angle.Target_Yaw;
    data2send[2] = 100.0f * Sides_PID.err;

    data2send[3] = Auto_spd.ID * 10.0f;
    data2send[4] = gait_param.std_step_length * 100.0f;
    //    data2send[4] = Auto_spd.stage_yaw;
    //    data2send[5] = leg[2].Foot_Target.p.z;

    //    data2send[6] = leg[3].Foot_Target.p.x;
    //    data2send[7] = leg[3].Foot_Target.p.z;

    //    data2send[8] = gait_param.Leg_State[0];
    /*

    data2send[0] = leg[0].Foot_Target.p.x;
    data2send[1] = leg[0].Foot_Target.p.z;

    data2send[2] = leg[1].Foot_Target.p.x;
    data2send[3] = leg[1].Foot_Target.p.z;

    data2send[4] = leg[2].Foot_Target.p.x;
    data2send[5] = leg[2].Foot_Target.p.z;

    data2send[6] = leg[3].Foot_Target.p.x;
    data2send[7] = leg[3].Foot_Target.p.z;

    data2send[8] = gait_param.Leg_State[0];
    */
    /*
      data2send[0] = leg[1].Foot.p.x;
      data2send[1] = leg[1].Foot_Target.p.x;

      data2send[2] = leg[1].Foot.p.z;
      data2send[3] = leg[1].Foot_Target.p.z;

      data2send[4] = motor[3].measure.T / 100.0f;
      data2send[5] = leg[1].Motor_Torque.Alpha / 100.0f;

      data2send[6] = motor[2].measure.T / 100.0f;
      data2send[7] = leg[1].Motor_Torque.Beta / 100.0f;

          data2send[8] = sqrtf(leg[1].Foot.p.x * leg[1].Foot.p.x + leg[1].Foot.p.z * leg[1].Foot.p.z);
  */

    //    data2send[8] = leg[1].VMC_Output_Force.x / 1000.0f;
    //    data2send[9] = leg[1].VMC_Output_Force.z / 1000.0f;
    //		data2send[9] = Jump_Param[1].Jump_Theta[0];

    /*
    data2send[0] = gait_param.step_length[0];
    data2send[1] = gait_param.step_length[1];

    data2send[2] = gait_param.step_length[2];
    data2send[3] = gait_param.step_length[3];

    data2send[4] = gait_param.VMC_Euler_Walk.Kp_yaw * (Euler_Angle.Target_Yaw - Euler_Angle.Yaw);
    data2send[5] = gait_param.VMC_Euler_Walk.Kw_yaw * (Euler_Angle.Last_Yaw - Euler_Angle.Yaw);
    data2send[6] = gait_param.VMC_Euler_Walk.Kp_yaw * (Euler_Angle.Target_Yaw - Euler_Angle.Yaw)-
                                                             gait_param.VMC_Euler_Walk.Kw_yaw * (Euler_Angle.Last_Yaw - Euler_Angle.Yaw);
    data2send[7] = gait_param.Leg_State[0];*/

    /*
        data2send[0] = leg[0].Foot.p.x;
        data2send[1] = leg[0].Foot.p.x;

        data2send[2] = leg[1].Foot.p.x;
        data2send[3] = leg[1].Foot.p.x;

        data2send[4] = leg[2].Foot.p.x;
        data2send[5] = leg[2].Foot.p.x;

        data2send[6] = leg[3].Foot.p.x;
        data2send[7] = leg[3].Foot.p.x;
    */
    /*
    data2send[0] = leg[2].Foot.p.x;
    data2send[1] = leg[2].Target_p.x;

    data2send[2] = leg[2].Foot.p.x;
    data2send[3] = leg[2].Target_p.z;

    data2send[4] = leg[2].VMC_Output_Force.x / 100.0f;
    data2send[5] = leg[2].VMC_Output_Force.z / 100.0f;

    data2send[6] = motor[5].measure.T;
    data2send[7] = leg[2].Motor_Torque.Alpha;

    data2send[8] = motor[4].measure.T;
    data2send[9] = leg[2].Motor_Torque.Beta;
    */
    extern Jump_VMC_Param Jump_VMC;

    //        data2send[0] = leg[1].Foot.p.x;
    //        data2send[1] = leg[1].Target_p.x;

    //        data2send[2] = leg[1].Foot.p.x;
    //        data2send[3] = leg[1].Target_p.z;

    //        data2send[4] = leg[1].VMC_Output_Force.x / 100.0f;
    //        data2send[5] = leg[1].VMC_Output_Force.z / 100.0f;

    //        data2send[6] = motor[3].measure.T;
    //        data2send[7] = leg[1].Motor_Torque.Alpha;

    ////        data2send[8] = motor[2].measure.T;
    ////        data2send[9] = leg[1].Motor_Torque.Beta;
    //        data2send[8] = leg[1].Motor.Pos.Beta;
    //        data2send[9] = leg[0].Motor.Pos.Beta;

    //    data2send[6] = Jump_VMC.VMC_Cushion[4].Kw_x * (0 - leg[1].now.Foot.v.x);
    //		data2send[7] = Jump_VMC.VMC_Cushion[4].Kw_z * (0 - leg[1].now.Foot.v.z);
    /*
    data2send[0] = leg[0].Foot.p.x;
    data2send[1] = leg[0].Target_p.x;

    data2send[2] = leg[0].Foot.p.x;
    data2send[3] = leg[0].Target_p.z;

    data2send[4] = Action_Param.VMC_Swing_Stand[0].differ_x / 1000.0f;
    data2send[5] = Action_Param.VMC_Swing_Stand[0].differ_z / 1000.0f;

    data2send[6] = leg[0].VMC_Output_Force.x / 1000.0f;
    data2send[7] = leg[0].VMC_Output_Force.z / 1000.0f;

    data2send[8] = Action_Param.VMC_Swing_Stand[0].err_x;
    data2send[9] = Action_Param.VMC_Swing_Stand[0].last_err_x;
    */
    /*
        data2send[0] = leg[0].Foot.p.x;
        data2send[1] = leg[0].Foot_Target.p.x;

        //    data2send[0] = wait_for_jump_flag;
        //    data2send[1] = memory_flag;

        data2send[2] = leg[0].Foot.p.z;
        data2send[3] = leg[0].Foot_Target.p.z;

        //    data2send[0] = leg[3].Foot.p.x;
        //    data2send[1] = Action_Param.Init_Pos[3].x;
        //
        //    data2send[2] = leg[3].Foot.p.z;
        //    data2send[3] = Action_Param.Init_Pos[3].z;

        data2send[4] = leg[3].Foot.p.x;
        data2send[5] = leg[3].Foot_Target.p.x;

        data2send[6] = leg[3].Foot.p.z;
        data2send[7] = leg[3].Foot_Target.p.z;

        data2send[8] = GetNowDogState();
        data2send[9] = GetDogWhetherChangeAble() sqrtf(leg[3].Foot.p.x * leg[3].Foot.p.x + leg[3].Foot.p.z * leg[3].Foot.p.z)*/
    ;

    /*
        data2send[0] = leg[2].Foot.p.x;
        data2send[1] = leg[2].Target_p.x;

        data2send[2] = leg[2].Foot.p.x;
        data2send[3] = leg[2].Target_p.z;

        data2send[4] = leg[3].Foot.p.x;
        data2send[5] = leg[3].Target_p.x;

        data2send[6] = leg[3].Foot.p.x;
        data2send[7] = leg[3].Target_p.z;

        data2send[8] = gait_param.Leg_State[2];
    */
    /*
    data2send[0] = leg[0].VMC_Output_Force.x;
    data2send[1] = leg[0].VMC_Output_Force.z;

    data2send[2] = leg[1].VMC_Output_Force.x;
    data2send[3] = leg[1].VMC_Output_Force.z;

    data2send[4] = leg[2].VMC_Output_Force.x;
    data2send[5] = leg[2].VMC_Output_Force.z;

    data2send[6] = leg[3].VMC_Output_Force.x;
    data2send[7] = leg[3].VMC_Output_Force.z;
    */
    /*
    data2send[0] = leg[0].Motor_Torque.Alpha;
    data2send[1] = leg[0].Motor_Torque.Beta;

    data2send[2] = leg[1].Motor_Torque.Alpha;
    data2send[3] = leg[1].Motor_Torque.Beta;

    data2send[4] = leg[2].Motor_Torque.Alpha;
    data2send[5] = leg[2].Motor_Torque.Beta;

    data2send[6] = leg[3].Motor_Torque.Alpha;
    data2send[7] = leg[3].Motor_Torque.Beta;
    */
    /*
    data2send[1] = leg[0].VMC_Stance.Kp_yaw * (0 - Euler_Angle.Yaw) + leg[0].VMC_Stance.Kw_yaw * (Euler_Angle.Last_Yaw - Euler_Angle.Yaw);
    data2send[6] = Action_Param.Init_Pos[0].x;
    data2send[7] = Action_Param.Init_Pos[0].z;
    */
    Usart_Send_To_Show32(&huart2, data2send);
}
