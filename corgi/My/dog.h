#ifndef _DOG_H
#define _DOG_H
#include "stdint.h"
#include "fsm.h"
#define RADIN_TO_REG 57.296f
#define REG_TO_RADIN 0.0174533f
// 角度制转换为弧度制系数
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
    Point p; // 足端当前位置
    Point v; // 足端当前速度
} foot;

typedef struct // 这个结构体里面的变量在串口接收中断里面更新
{
    float Yaw;   // 偏航
    float Pitch; // 俯仰
    float Roll;  // 横滚

    float Last_Pitch; // 俯仰
    float cnt_Pitch;

    float Last_Yaw;   // 上一刻偏航角,PID用
    float Target_Yaw; // 目标偏航角,PID用

    float Gamepad_Last_Yaw; // 手柄控制时上一刻偏航角,中间变量
    float Update_Yaw;       // 重置陀螺仪零点
    float test_Pitch;       // 测试用,用完删
} Euler;

typedef struct
{
    float Kp_x; // x方向Kp
    float Kd_x; // x方向Kd
    float k_x;  // x方向不完全微分系数

    float Kp_z; // z方向Kp
    float Kd_z; // z方向Kd
    float k_z;  // z方向不完全微分系数

    float Feedforward_x; // 前馈x
    float Feedforward_z; // 前馈z

    float err_x;         // x方向当前误差
    float last_err_x;    // x方向上一时刻误差
    float differ_x;      // x方向当前微分项大小
    float last_differ_x; // x方向上一时刻微分项大小

    float err_z;         // z方向当前误差
    float last_err_z;    // z方向上一时刻误差
    float differ_z;      // z方向当前微分项大小
    float last_differ_z; // z方向上一时刻微分项大小
} PD;

typedef struct
{
    float x[24]; // 滤波器缓冲区
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
    uint8_t id;              // 腿id(0~3)
    float rod[4];            // 四连杆长度
    float start_Alpha;       // 上电时规定的Alpha角度(rad)
    float start_Beta;        // 上电时规定的Beta角度
    float start_motor_Alpha; // 上电时Alpha角对应的电机机械角度
    float start_motor_Beta;  // 上电时Beta角对应的电机机械角度
    uint8_t motor_Alpha_id;  // 对应Alpha角电机的id
    uint8_t motor_Beta_id;   // 对应Beta角电机的id
    int8_t motor_Alpha_dir;  // 对应Alpha角电机的旋转方向,逆时针正为1,顺时针正为-1
    int8_t motor_Beta_dir;   // 对应Beta角电机的旋转方向,逆时针正为1,顺时针正为-1

    float last_a1; // 运动学逆解的中间变量
    float true_a1; // 运动学逆解的中间变量
} Param;

typedef struct
{
    float Tfb; // 前馈力
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
    Param const_param;                 // 记录一些中间参数变量和常数变量
    Ctrl_State ctrl_state;             // 控制标志位,只有leg[0]的有用
    Pitch_Ctrl_State pitch_ctrl_state; // 腿在俯仰方向是否和腿保持相对静止,Follow_Ground为跟随地面俯仰角,Follow_Dog为相对狗保持不动
    Kine Joint;                        // 电机当前角度(关节坐标系),电机当前角速度(关节坐标系,逆时针为正)
    Kine Motor;                        // 电机当前角度(电机坐标系),电机当前角速度(电机坐标系,逆时针为正)
    foot Foot;                         // 足端当前位置,足端当前速度

    foot Foot_Target;       // 足端目标位置
    Order Motor_Torque;     // 电机目标输出力矩(VMC输出力矩,使用单力矩输出模式)
    Point VMC_Output_Force; // VMC经过计算后算出的目标输出力矩(关节坐标系)

    pctrl Pctrl; // 位置控制相关
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
