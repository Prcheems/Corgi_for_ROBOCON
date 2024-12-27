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

// 初始位置是指2pi/9中的绝对位置,输出轴一圈有九个绝对位置,根据初始摆放位置不同,电机可能转到别的绝对位置上去,尽量保证每次上电时电机初始角度一致
// 高度最小10cm
// id        //motor
// 10  10    10   76

// 10  10    32   54

// 腿长最短0.14667m 最长0.4048m

LEG leg[4];
Euler Euler_Angle;
extern A1_Motor_Struct motor[8]; // 电机
extern float data2send[10];
/* ------------------------------------------运动学分析------------------------------------------------ */

/*

这部分有三组转换函数。分别是：
    关节->足端：Kinematic_Solution
    足端->关节：Kinematic_Inversion
    电机->足端：Motor_Convert_To_Foot
    足端->电机：Foot_Convert_To_Motor
    电机->关节：Motor_Convert_To_Joint
    关节->电机：Joint_Convert_To_Motor
    关节和足端坐标系的转换就是运动学的正逆解,但是注意关节的坐标量和电机实际编码器上的量有一些区别,具体区别看下面注释
*/

/**
 * @brief 四连杆运动学正解,输入电机转角,输出足端位置；输入电机转速,输出足端速度
 * @param param 狗单腿结构体里的一些不会变的定参
 * @param Joint 两电机在关节坐标系的速度 位置
 * @param Foot 解算出的足端速度 位置结构体
 */
void Kinematic_Solution(Param *param, foot *Foot, Kine *Joint) // 其实就是Motor_Covert_To_Foot,关节坐标系到足端坐标系
{
    float xb, xd, zb, zd;
    float A, B, C;
    float vbx, vbz, vdx, vdz;
    float sin_Phi1, sin_Phi2, cos_Phi1, cos_Phi2, w_Phi1;

    // 根据电机转角计算足端位置
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

    // 根据电机角速度计算足端速度
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
    // 根据足端位置计算关节电机转角
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
 * @brief 四连杆运动学逆解,输入足端位置,输出电机转角；输入足端速度,输出电机转速
 * @param param 狗单腿结构体里的一些不会变的定参
 * @param Joint 两电机在关节坐标系的速度 位置
 * @param Foot 解算出的足端速度 位置结构体
 */
void Kinematic_Inversion(Param *param, foot *Foot, Kine *Joint)
{
    Kinematic_Inversion_Update(param, Foot, Joint);

    // 根据足端速度计算关节电机角速度
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
 * @brief 四连杆运动学逆解,输入足端位置,输出电机转角；输入足端速度,输出电机转速
 * @param leg 狗单腿结构体对应的所有可变参数
 * @param param 狗单腿结构体里的一些不会变的定参
 * @param VMC_Output 腿坐标系下x,z方向输出的虚拟力大小
 * @param Motor_Torque 关节电机输出力矩
 * @warning 使用这个函数前必须用Get_Now_Pos_Spd函数获得关节坐标系的电机位置
 */
void VMC_Jacobi_Matrix(LEG *leg, Param *param, Point *VMC_Output, Order *Motor_Torque)
{
    // 算出Phi1,Phi2,和运动学正解的一模一样
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

    // 定义雅可比矩阵
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
 * @brief  因为宇树电机没有绝对编码器,上电时编码器的值就是0。所以要做一步电机坐标系和关节坐标系的转化
           电机坐标系下角度 = 电机旋转方向*电机已经旋转的角度+电机坐标系下初始角度
           电机旋转方向：面朝电机,逆时针旋转记为+,顺时针记为-
           电机已经旋转的角度=关节坐标系下电机当前角度-关节坐标系下电机初始角度
           电机坐标系下初始角度：把腿摆到初始位置(腿刚好卡住两根固定柱),电机上电,此时电机的回传值
 * @param  param 狗单腿结构体里的一些不会变的定参
 * @param  Joint 两电机在关节坐标系的速度 位置
 * @param  Motor 两电机在电机坐标系的速度 位置
 */
void Joint_Convert_To_Motor(Param *param, Kine *Joint, Kine *Motor) // 关节坐标系下角度与角速度转换到电机坐标系下角度与角速度
{
    Motor->Pos.Alpha = param->motor_Alpha_dir * (Joint->Pos.Alpha - param->start_Alpha) + param->start_motor_Alpha;
    Motor->Pos.Beta = param->motor_Beta_dir * (Joint->Pos.Beta - param->start_Beta) + param->start_motor_Beta;
    Motor->Spd.Alpha = param->motor_Alpha_dir * Joint->Spd.Alpha;
    Motor->Spd.Beta = param->motor_Beta_dir * Joint->Spd.Beta;
}

void Motor_Convert_To_Joint(Param *param, Kine *Joint, Kine *Motor) // 电机坐标系下角度与角速度转换到关节坐标系下角度与角速度
{
    Joint->Pos.Alpha = param->motor_Alpha_dir * (Motor->Pos.Alpha - param->start_motor_Alpha) + param->start_Alpha;
    Joint->Pos.Beta = param->motor_Beta_dir * (Motor->Pos.Beta - param->start_motor_Beta) + param->start_Beta;
    Joint->Spd.Alpha = param->motor_Alpha_dir * Motor->Spd.Alpha;
    Joint->Spd.Beta = param->motor_Beta_dir * Motor->Spd.Beta;
}

void Foot_Convert_To_Motor(Param *param, foot *Foot, Kine *Joint, Kine *Motor) // 足端坐标系下角度与角速度转换到电机坐标系下角度与角速度
{
    Kinematic_Inversion(param, Foot, Joint);
    Joint_Convert_To_Motor(param, Joint, Motor);
}

void Motor_Convert_To_Foot(Param *param, foot *Foot, Kine *Joint, Kine *Motor) // 电机坐标系下角度与角速度转换到足端坐标系下角度与角速度
{
    Motor_Convert_To_Joint(param, Joint, Motor);
    Kinematic_Solution(param, Foot, Joint);
}

/* ------------------------------------------通讯函数------------------------------------------------ */

/**
 * @brief  力控发送目标力矩
 * @param  leg 腿大结构体
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
 * @brief  位控发送目标速度位置
 * @param  leg 腿大结构体
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

/* ------------------------------------------电机的初始化------------------------------------------------ */

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

void leg_init() // 更换电机只需改此部分
{
    uint8_t id = 0;
    // 腿0
    id = 0;
    leg[id].const_param.motor_Alpha_id = 1;
    leg[id].const_param.motor_Alpha_dir = 1;
    leg[id].const_param.start_Alpha = -116.41f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Alpha = 0.3352;

    leg[id].const_param.motor_Beta_id = 0;
    leg[id].const_param.motor_Beta_dir = -1;
    leg[id].const_param.start_Beta = -203.01f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Beta = 0.3107;

    // 腿1
    id = 1;
    leg[id].const_param.motor_Alpha_id = 1;
    leg[id].const_param.motor_Alpha_dir = 1;
    leg[id].const_param.start_Alpha = -116.41f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Alpha = 0.1255;

    leg[id].const_param.motor_Beta_id = 0;
    leg[id].const_param.motor_Beta_dir = -1;
    leg[id].const_param.start_Beta = -203.01f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Beta = 0.1488;

    // 腿2
    id = 2;
    leg[id].const_param.motor_Alpha_id = 1;
    leg[id].const_param.motor_Alpha_dir = 1;
    leg[id].const_param.start_Alpha = 23.01f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Alpha = 0.5963;

    leg[id].const_param.motor_Beta_id = 0;
    leg[id].const_param.motor_Beta_dir = -1;
    leg[id].const_param.start_Beta = -63.59f * REG_TO_RADIN;
    leg[id].const_param.start_motor_Beta = 0.0851;

    // 腿3
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

        // 腿四连杆长度
        leg[i].const_param.rod[0] = 0.175f;
        leg[i].const_param.rod[1] = 0.3f;
        leg[i].const_param.rod[2] = 0.175f;
        leg[i].const_param.rod[3] = 0.3f;

        // 初始化计算参数
        leg[i].Motor.Pos.Alpha = motor[leg[i].const_param.motor_Alpha_id + 2 * i].measure.Pos;
        leg[i].Motor.Pos.Beta = motor[leg[i].const_param.motor_Beta_id + 2 * i].measure.Pos;

        // 计算足端初始位置
        Get_Now_Pos_Spd(i);
    }
}

/* ------------------------------------------基础动作------------------------------------------------ */

// 腿坐标系以腿0为标准,即x为正时,腿2、3坐标系中x为负

/*

关于代码体系中的时间安排：
(1)1ms进一次定时器中断->tick+1,也就是说tick记录了自打开定时器开始程序运行的时间,将作为狗运动的节律一直被记录。
(2)FSM的步骤：执行一次进入函数->执行n次执行函数,直到完成执行动作->执行一次退出函数
(3)也就是说我在进入函数中获得开始执行动作时的系统时间tick_start,然后开始执行stand_run函数,
随着不断的执行,GetTick()函数的返回值不断增大,我把执行动作中的系统时间和刚开始执行动作的系统时间做差,
记录在了t变量里面,那么t就是一个一个系统执行时间的离散点。比如设定stand动作执行2s,
那么t的计数就是0,1,2,3,4,5,......,2000,从而生成了关于时间均等分布的点,用来生成轨迹
*/

void Get_Now_Pos_Spd(uint8_t legid)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
    {
        switch (legid)
        {
        case 0:
            leg[0].Motor.Pos.Alpha = motor[1].measure.Pos;
            leg[0].Motor.Pos.Beta = motor[0].measure.Pos;
            leg[0].Motor.Spd.Alpha = motor[1].measure.W;
            leg[0].Motor.Spd.Beta = motor[0].measure.W;

            // 电机坐标系转化到足端,更新足端位置
            Motor_Convert_To_Joint(&leg[0].const_param, &leg[0].Joint, &leg[0].Motor);
            Kinematic_Solution(&leg[0].const_param, &leg[0].Foot, &leg[0].Joint);

            // 更新运动学逆解算出的电机转角
            if (Get_Now_Ctrl_State() == Force_Ctrl)
                Kinematic_Inversion_Update(&leg[0].const_param, &leg[0].Foot, &leg[0].Joint);

            break;

        case 1:
            leg[1].Motor.Pos.Alpha = motor[3].measure.Pos;
            leg[1].Motor.Pos.Beta = motor[2].measure.Pos;
            leg[1].Motor.Spd.Alpha = motor[3].measure.W;
            leg[1].Motor.Spd.Beta = motor[2].measure.W;

            // 电机坐标系转化到足端,更新足端位置
            Motor_Convert_To_Joint(&leg[1].const_param, &leg[1].Joint, &leg[1].Motor);
            Kinematic_Solution(&leg[1].const_param, &leg[1].Foot, &leg[1].Joint);

            // 更新运动学逆解算出的电机转角
            if (Get_Now_Ctrl_State() == Force_Ctrl)
                Kinematic_Inversion_Update(&leg[1].const_param, &leg[1].Foot, &leg[1].Joint);

            break;

        case 2:
            leg[2].Motor.Pos.Alpha = motor[5].measure.Pos;
            leg[2].Motor.Pos.Beta = motor[4].measure.Pos;
            leg[2].Motor.Spd.Alpha = motor[5].measure.W;
            leg[2].Motor.Spd.Beta = motor[4].measure.W;

            // 电机坐标系转化到足端,更新足端位置
            Motor_Convert_To_Joint(&leg[2].const_param, &leg[2].Joint, &leg[2].Motor);
            Kinematic_Solution(&leg[2].const_param, &leg[2].Foot, &leg[2].Joint);

            // 更新运动学逆解算出的电机转角
            if (Get_Now_Ctrl_State() == Force_Ctrl)
                Kinematic_Inversion_Update(&leg[2].const_param, &leg[2].Foot, &leg[2].Joint);

            break;

        case 3:
            leg[3].Motor.Pos.Alpha = motor[7].measure.Pos;
            leg[3].Motor.Pos.Beta = motor[6].measure.Pos;
            leg[3].Motor.Spd.Alpha = motor[7].measure.W;
            leg[3].Motor.Spd.Beta = motor[6].measure.W;

            // 电机坐标系转化到足端,更新足端位置
            Motor_Convert_To_Joint(&leg[3].const_param, &leg[3].Joint, &leg[3].Motor);
            Kinematic_Solution(&leg[3].const_param, &leg[3].Foot, &leg[3].Joint);

            // 更新运动学逆解算出的电机转角
            if (Get_Now_Ctrl_State() == Force_Ctrl)
                Kinematic_Inversion_Update(&leg[3].const_param, &leg[3].Foot, &leg[3].Joint);

            break;
        }
    }
}

/**
 * @brief 旋转坐标变换
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
 * @brief PD项计算虚拟力输出
 * @param Param PD参数
 * @param Target_Pos 足端在腿坐标系下在x,z方向上的目标坐标,由直线/摆线生成器得到
 * @param Now_Pos 足端在腿坐标系下在x,z方向上的当前坐标,由电机反馈值+运动学正解得到
 * @param VMC_Output 对于单个支撑腿,在腿坐标系下x,z方向需要输出的虚拟力大小
 */
void PD_Control(PD *Param, Point *Target_Pos, Point *Now_Pos, Point *VMC_Output)
{
    // 计算误差error
    Param->err_x = Target_Pos->x - Now_Pos->x;
    Param->err_z = Target_Pos->z - Now_Pos->z;

    // 计算不完全D项
    Param->differ_x = Param->Kd_x * (1 - Param->k_x) * (Param->err_x - Param->last_err_x) + Param->k_x * Param->last_differ_x;
    Param->differ_z = Param->Kd_z * (1 - Param->k_z) * (Param->err_z - Param->last_err_z) + Param->k_z * Param->last_differ_z;

    // 计算输出虚拟力
    VMC_Output->x = Param->Kp_x * Param->err_x + Param->differ_x + Param->Feedforward_x;
    VMC_Output->z = Param->Kp_z * Param->err_z + Param->differ_z + Param->Feedforward_z;

    // 传递参数
    Param->last_err_x = Param->err_x;
    Param->last_differ_x = Param->differ_x;
    Param->last_err_z = Param->err_z;
    Param->last_differ_z = Param->differ_z;
}

/**
 * @brief 足端走一条直线
 * @param legid 输入腿的id,判断是哪只腿走直线
 * @param start_x,start_z 直线始端
 * @param end_x,end_z 直线末端
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @param Swing PD控制摆动相参数
 */
void line(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float T, float t, PD *Swing)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
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
 * @brief 足端走一条摆线
 * @param legid 输入腿的id,判断是哪只腿走直线
 * @param start_x,start_z 摆线始端
 * @param end_x 摆线末端
 * @param max_z 摆线顶点z坐标
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @param Swing PD控制摆动相参数
 */
void Cycloid(uint8_t legid, float start_x, float start_z, float end_x, float max_z, float T, float t, PD *Swing)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
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
这里说一下bezier曲线系的定义:以(start_x,start_z)为原点,狗整体的正向为x轴正向,竖直向上为z轴正向
*/

/**
 * @brief 足端走贝塞尔曲线,摆动相使用
 * @param legid 输入腿的id,判断是哪只腿走摆线(涉及速度取正取负)
 * @param start_x,start_z 腿坐标系下曲线始端
 * @param end_x,end_z 腿坐标系下曲线末端
 * @param max_z 曲线的最大高度,如果不限制写0,向腿坐标系z轴正向摆为-,向腿坐标系z轴负向摆为+
 * @param Delta_x,Delta_z 在腿坐标系下,贝塞尔曲线的原点坐标
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @param Swing PD控制摆动相参数
 */
void Bezier(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t, PD *Swing)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
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
 * @brief 足端走贝塞尔曲线,walk状态支撑相使用
 * @param legid 输入腿的id,判断是哪只腿走摆线(涉及速度取正取负)
 * @param start_x,start_z 腿坐标系下曲线始端
 * @param end_x,end_z 腿坐标系下曲线末端
 * @param max_z 曲线的最大高度,如果不限制写0,向腿坐标系z轴正向摆为-,向腿坐标系z轴负向摆为+
 * @param Delta_x,Delta_z 在腿坐标系下,贝塞尔曲线的原点坐标
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @param Stance 支撑相支撑的参数
 */
void Bezier_Stance(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t, PD *Stance)
{
    if (legid <= 3 && legid >= 0) // 严防数组越界
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
 * @brief 直线生成器
 * @param legid 输入腿的id,判断是哪只腿走直线
 * @param start_x,start_z 直线始端
 * @param end_x,end_z 直线末端
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @return Target_Pos 目标x,z
 */
Point line_Generator(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float T, float t)
{
    Point Target_Pos = {0, 0, 0}; // 初始化目标xz
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
 * @brief 摆线生成器,-s表示反方向
 * @param legid 输入腿的id,判断是哪只腿走摆线(涉及速度取正取负)
 * @param start_x,start_z 摆线始端
 * @param s 摆线在x方向长度
 * @param h 摆线在z方向最大高度
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @return Target_Pos 目标x,z
 */
Point Cycloid_Generator(uint8_t legid, float start_x, float start_z, float s, float h, float T, float t)
{
    Point Target_Pos = {0, 0, 0}; // 初始化目标xz
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
 * @brief 贝塞尔曲线生成器
 * @param legid 输入腿的id
 * @param start_x,start_z 曲线始端
 * @param end_x,end_z 曲线末端
 * @param max_z 贝塞尔坐标系下曲线的最大高度,如果不限制写0
 * @param Delta_x,Delta_z 在腿坐标系下,贝塞尔曲线的原点坐标
 * @param T 做整个动作需要的时间
 * @param t 这个动作已经做的时间
 * @return Target_Pos 目标x,z
 */
Point Bezier_Generator(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t)
{
    // 初始化
    // 腿坐标系下的目标位置,起点终点坐标
    Point Target_Pos_leg = {0, 0, 0};
    Point Start_leg = {0, 0, 0};
    Point End_leg = {0, 0, 0};
    // 贝塞尔坐标系下的目标位置,起点终点坐标
    Point Target_Pos_Bezier = {0, 0, 0};
    Point Start_Bezier = {0, 0, 0};
    Point End_Bezier = {0, 0, 0};

    Start_leg.x = start_x;
    Start_leg.z = start_z;
    End_leg.x = end_x;
    End_leg.z = end_z;

    // 腿坐标系转化到贝塞尔曲线坐标系
    Bezier_Trans(legid, &Start_leg, &Start_Bezier, Delta_x, Delta_z);
    Bezier_Trans(legid, &End_leg, &End_Bezier, Delta_x, Delta_z);

    // 时间控制
    float Now_Discrete_Time_Point = t;
    if (t > T)
        Now_Discrete_Time_Point = T;

    float Std_Time = 0;

    if (T != 0)
        Std_Time = Now_Discrete_Time_Point / T; // 标准化时间,范围0-1

    // 计算目标x
    Target_Pos_Bezier.x = (1.0f - Std_Time) * (1.0f - Std_Time) * (1.0f - Std_Time) * Start_Bezier.x +
                          3.0f * (1.0f - Std_Time) * (1.0f - Std_Time) * Std_Time * Start_Bezier.x +
                          3.0f * (1.0f - Std_Time) * Std_Time * Std_Time * End_Bezier.x +
                          Std_Time * Std_Time * Std_Time * End_Bezier.x;

    Target_Pos_Bezier.z = (1 - Std_Time) * (1.0f - Std_Time) * (1.0f - Std_Time) * Start_Bezier.z +
                          3.0f * (1.0f - Std_Time) * (1 - Std_Time) * Std_Time * (max_z * 0.09f / 0.0675f) +
                          3.0f * (1.0f - Std_Time) * Std_Time * Std_Time * (max_z * 0.09f / 0.0675f) +
                          Std_Time * Std_Time * Std_Time * End_Bezier.z;

    // 贝塞尔曲线坐标系转化到腿坐标系
    Bezier_Trans_Inv(legid, &Target_Pos_leg, &Target_Pos_Bezier, Delta_x, Delta_z);
    return Target_Pos_leg;
}

/**
 * @brief 腿坐标系转化到贝塞尔曲线坐标系
 * @param legid 输入腿的id
 * @param Foot 输入腿坐标系下足端位置坐标
 * @param Bezier 输出贝塞尔定义坐标系下足端位置坐标
 * @param Delta_x,Delta_z 在腿坐标系下,贝塞尔曲线的原点坐标
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
 * @brief 贝塞尔曲线坐标系转化到腿坐标系
 * @param legid 输入腿的id
 * @param Foot 输入腿坐标系下足端位置坐标
 * @param Bezier 输出贝塞尔定义坐标系下足端位置坐标
 * @param Delta_x,Delta_z 在腿坐标系下,贝塞尔曲线的原点坐标
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
 * @brief 限幅平均滤波法,滤去高频噪声(抖动),且避免剧烈突变(剧烈响声+拉扯机构)
 * @param Input_Value 输入解算值
 * @param filter_buf 滤波缓冲区数组头地址
 * @param Max_AMP 允许最大突变量
 * @return 经过处理后的输出值
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
 * @brief 改变俯仰角修正方式
 * @param state
 * @param legid
 * @param Pitch 输出贝塞尔定义坐标系下足端位置坐标
 * @param Delta_x,Delta_z 在腿坐标系下,贝塞尔曲线的原点坐标
 */
void Change_Single_Pitch_State(Pitch_Ctrl_State state, uint8_t legid)
{
    leg[legid].pitch_ctrl_state = state;
}

float data2send[10];
extern action_param Action_Param;
extern float wait_for_jump;        // 等待俯仰角符合条件时记录的中间时刻
extern uint8_t wait_for_jump_flag; // 标志位,0表示尚未记录时刻,1表示已经记录时刻,但俯仰角尚未符合条件,2表示可以进入第二次跳跃
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
