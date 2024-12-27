#ifndef _FMC_H

#define _FMC_H
#include "stm32h7xx.h"
typedef enum // ������ö���������˹���״̬
{
    Start = 0,   // ��ʼģʽ
    Keep,        // ���ֹؽ�λ��
    Squat,       // ����
    Stand,       // վ��ģʽ
    Walk,        // ����
    Jump,        // ��Ծ
    Backflip,    // ��շ�
    CrossLeg,    // ����
    Line,        // �����ֱ��,���ڵ�������
    JumpHurdles, // ����
    Test1,
    Test2,
    Test3,
} DOG_State;

typedef struct
{
    void (*enter)();     // ����һ��enter�ĺ���ָ��,ָ��״̬�����뺯���ĵ�ַ
    void (*run)();       // ����һ��run�ĺ���ָ��,ָ��״̬��ִ�к����ĵ�ַ
    void (*exit)();      // ����һ��exit�ĺ���ָ��,ָ��״̬���˳������ĵ�ַ
    DOG_State now_state; // ��DOG_State��ö������¼�����������һ��,��ǰ,֮���״̬
    DOG_State next_state;
    DOG_State last_state;
    FunctionalState change_able; // FunctionalState��һ��0-1��enum��,�жϵ�ǰ״̬�Ƿ���Ըı�,�Ƿ�ִ�е�ǰ״̬�ĺ���
    FunctionalState execute_able;
} FSM_State;

void DogFSM(void);
void ChangeDogState(DOG_State x);
DOG_State GetNowDogState(void);
DOG_State GetLastDogState(void);
DOG_State GetNextDogState(void);
void SetDogChangeAble(FunctionalState able);
FunctionalState GetDogWhetherChangeAble(void);
// ��ȡ�Ƿ�Ҫ�л�״̬
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
