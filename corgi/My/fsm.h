#ifndef _FMC_H

#define _FMC_H
#include "stm32h7xx.h"
typedef enum // 这里用枚举量定义了狗的状态
{
    Start = 0,   // 初始模式
    Keep,        // 保持关节位置
    Squat,       // 蹲着
    Stand,       // 站立模式
    Walk,        // 行走
    Jump,        // 跳跃
    Backflip,    // 后空翻
    CrossLeg,    // 跨腿
    Line,        // 足端走直线,用于调整重心
    JumpHurdles, // 跨栏
    Test1,
    Test2,
    Test3,
} DOG_State;

typedef struct
{
    void (*enter)();     // 创建一个enter的函数指针,指向状态机进入函数的地址
    void (*run)();       // 创建一个run的函数指针,指向状态机执行函数的地址
    void (*exit)();      // 创建一个exit的函数指针,指向状态机退出函数的地址
    DOG_State now_state; // 用DOG_State的枚举量记录四足机器人上一次,当前,之后的状态
    DOG_State next_state;
    DOG_State last_state;
    FunctionalState change_able; // FunctionalState是一个0-1的enum量,判断当前状态是否可以改变,是否执行当前状态的函数
    FunctionalState execute_able;
} FSM_State;

void DogFSM(void);
void ChangeDogState(DOG_State x);
DOG_State GetNowDogState(void);
DOG_State GetLastDogState(void);
DOG_State GetNextDogState(void);
void SetDogChangeAble(FunctionalState able);
FunctionalState GetDogWhetherChangeAble(void);
// 获取是否要切换状态
FunctionalState GetDogWhetherExecuteAble(void);

void start_enter(void);
void start_exit(void);
void start_run(void);

void keep_enter(void);
void keep_exit(void);
void keep_run(void);

void squat_enter(void);
void squat_exit(void);
void squat_run(void);

void stand_enter(void);
void stand_exit(void);
void stand_run(void);

void walk_enter(void);
void walk_exit(void);
void walk_run(void);

void jump_enter(void);
void jump_exit(void);
void jump_run(void);

void backflip_enter(void);
void backflip_exit(void);
void backflip_run(void);

void crossleg_enter(void);
void crossleg_exit(void);
void crossleg_run(void);

void line_enter(void);
void line_exit(void);
void line_run(void);

void test1_enter(void);
void test1_exit(void);
void test1_run(void);

void test2_enter(void);
void test2_exit(void);
void test2_run(void);

void test3_enter(void);
void test3_exit(void);
void test3_run(void);

void jump_hurdles_enter(void);
void jump_hurdles_exit(void);
void jump_hurdles_run(void);

#endif
