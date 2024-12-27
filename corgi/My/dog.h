#ifndef _DOG_H
#define _DOG_H
#include "stdint.h"
#include "fsm.h"
#define RADIN_TO_REG 57.296f
#define REG_TO_RADIN 0.0174533f
// �Ƕ���ת��Ϊ������ϵ��
#define CHANGE_TO_RADIAN (0.01745329251994f)
#define PI 3.14159265f

typedef struct
{
    float x;
    float y;
    float z;
} Point;

typedef struct
{
    float Alpha;
    float Beta;
} Order;

typedef struct
{
    Order Pos;
    Order Spd;
} Kine;

typedef struct
{
    Point p; // ��˵�ǰλ��
    Point v; // ��˵�ǰ�ٶ�
} foot;

typedef struct // ����ṹ������ı����ڴ��ڽ����ж��������
{
    float Yaw;   // ƫ��
    float Pitch; // ����
    float Roll;  // ���

    float Last_Pitch; // ����
    float cnt_Pitch;

    float Last_Yaw;   // ��һ��ƫ����,PID��
    float Target_Yaw; // Ŀ��ƫ����,PID��

    float Gamepad_Last_Yaw; // �ֱ�����ʱ��һ��ƫ����,�м����
    float Update_Yaw;       // �������������
    float test_Pitch;       // ������,����ɾ
} Euler;

typedef struct
{
    float Kp_x; // x����Kp
    float Kd_x; // x����Kd
    float k_x;  // x������ȫ΢��ϵ��

    float Kp_z; // z����Kp
    float Kd_z; // z����Kd
    float k_z;  // z������ȫ΢��ϵ��

    float Feedforward_x; // ǰ��x
    float Feedforward_z; // ǰ��z

    float err_x;         // x����ǰ���
    float last_err_x;    // x������һʱ�����
    float differ_x;      // x����ǰ΢�����С
    float last_differ_x; // x������һʱ��΢�����С

    float err_z;         // z����ǰ���
    float last_err_z;    // z������һʱ�����
    float differ_z;      // z����ǰ΢�����С
    float last_differ_z; // z������һʱ��΢�����С
} PD;

typedef struct
{
    float x[24]; // �˲���������
    float z[24];
    float Max_AMP_x;
    float Max_AMP_z;
    float res_x;
    float res_z;
    float first_order_value_x;
    float first_order_value_z;
    float first_order_last_value_x;
    float first_order_last_value_z;
} Fliter;

typedef struct
{
    Kine Joint_Target;
    Kine Motor_Target;
    foot Foot_Last_Target;
} pctrl;

typedef struct
{
    uint8_t id;              // ��id(0~3)
    float rod[4];            // �����˳���
    float start_Alpha;       // �ϵ�ʱ�涨��Alpha�Ƕ�(rad)
    float start_Beta;        // �ϵ�ʱ�涨��Beta�Ƕ�
    float start_motor_Alpha; // �ϵ�ʱAlpha�Ƕ�Ӧ�ĵ����е�Ƕ�
    float start_motor_Beta;  // �ϵ�ʱBeta�Ƕ�Ӧ�ĵ����е�Ƕ�
    uint8_t motor_Alpha_id;  // ��ӦAlpha�ǵ����id
    uint8_t motor_Beta_id;   // ��ӦBeta�ǵ����id
    int8_t motor_Alpha_dir;  // ��ӦAlpha�ǵ������ת����,��ʱ����Ϊ1,˳ʱ����Ϊ-1
    int8_t motor_Beta_dir;   // ��ӦBeta�ǵ������ת����,��ʱ����Ϊ1,˳ʱ����Ϊ-1

    float last_a1; // �˶�ѧ�����м����
    float true_a1; // �˶�ѧ�����м����
} Param;

typedef struct
{
    float Tfb; // ǰ����
    float kp;
    float kw;
} Pos_motor;

typedef enum
{
    Force_Ctrl = 0,
    Pos_Ctrl,
} Ctrl_State;

typedef enum
{
    Follow_Ground = 0,
    Follow_Dog,
    Follow_Angle,
} Pitch_Ctrl_State;

typedef struct
{
    Param const_param;                 // ��¼һЩ�м���������ͳ�������
    Ctrl_State ctrl_state;             // ���Ʊ�־λ,ֻ��leg[0]������
    Pitch_Ctrl_State pitch_ctrl_state; // ���ڸ��������Ƿ���ȱ�����Ծ�ֹ,Follow_GroundΪ������温����,Follow_DogΪ��Թ����ֲ���
    Kine Joint;                        // �����ǰ�Ƕ�(�ؽ�����ϵ),�����ǰ���ٶ�(�ؽ�����ϵ,��ʱ��Ϊ��)
    Kine Motor;                        // �����ǰ�Ƕ�(�������ϵ),�����ǰ���ٶ�(�������ϵ,��ʱ��Ϊ��)
    foot Foot;                         // ��˵�ǰλ��,��˵�ǰ�ٶ�

    foot Foot_Target;       // ���Ŀ��λ��
    Order Motor_Torque;     // ���Ŀ���������(VMC�������,ʹ�õ��������ģʽ)
    Point VMC_Output_Force; // VMC��������������Ŀ���������(�ؽ�����ϵ)

    pctrl Pctrl; // λ�ÿ������
    //    Fliter Pos_Filter;
    //    Fliter Force_Filter;

} LEG;

extern Euler Euler_Angle;
extern LEG leg[4];
extern float data2send[10];

void Kinematic_Solution(Param *param, foot *Foot, Kine *Joint);
void Kinematic_Inversion(Param *param, foot *Foot, Kine *Joint);
void Joint_Convert_To_Motor(Param *param, Kine *Joint, Kine *Motor);
void Motor_Convert_To_Joint(Param *param, Kine *Joint, Kine *Motor);
void Motor_Convert_To_Foot(Param *param, foot *Foot, Kine *Joint, Kine *Motor);
void Foot_Convert_To_Motor(Param *param, foot *Foot, Kine *Joint, Kine *Motor);

void Leg_Initpos_Reset(void);
void leg_init(void);
void Set_Motor_Target_Torque(LEG *leg);
void Set_Motor_Target_Pos(LEG *leg);
void Get_Now_Pos_Spd(uint8_t legid);

void VMC_Jacobi_Matrix(LEG *leg, Param *param, Point *VMC_Output, Order *Motor_Torque);
void PD_Control(PD *Param, Point *Target_Pos, Point *Now_Pos, Point *VMC_Output);

Point line_Generator(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float T, float t);
Point Cycloid_Generator(uint8_t legid, float start_x, float start_z, float s, float h, float T, float t);
Point Bezier_Generator(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t);
void Bezier_Trans(uint8_t legid, Point *Foot, Point *Bezier, float Delta_x, float Delta_z);
void Bezier_Trans_Inv(uint8_t legid, Point *Foot, Point *Bezier, float Delta_x, float Delta_z);

void line(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float T, float t, PD *Swing);
void Cycloid(uint8_t legid, float start_x, float start_z, float end_x, float max_z, float T, float t, PD *Swing);
void Bezier(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t, PD *Swing);
void Bezier_Stance(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t, PD *Stance);

float VMC_Filter(float Input_Value, float filter_buf[], float Max_AMP, float first_order_value, float first_order_last_value);
void ano_tc(void);
void Coordinate_Transform(Point *Input, Point *Output, float Pitch, LEG *leg, uint8_t legid);
void Change_Dog_Pitch_State(Pitch_Ctrl_State state);
void Change_Single_Pitch_State(Pitch_Ctrl_State state, uint8_t legid);

#endif
