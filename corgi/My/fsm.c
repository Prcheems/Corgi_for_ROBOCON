#include "fsm.h"
#include "stdint.h"
#include "A1_motor.h"
#include "dog.h"
#include "timer.h"

// �����ͬ��.h�ﶨ���enum��һһ��Ӧ��,Ҳ����StateMap[keep]��Ӧ�ľ��Ǻ�keep״̬�йص���������ָ�롰keep_enter����keep_run����keep_exit��
FSM_State StateMap[] =
    {
        {start_enter, start_run, start_exit, Start, Start, Start, ENABLE, DISABLE},
        {keep_enter, keep_run, keep_exit},
        {squat_enter, squat_run, squat_exit},
        {stand_enter, stand_run, stand_exit},
        {walk_enter, walk_run, walk_exit},
        {jump_enter, jump_run, jump_exit},
        {backflip_enter, backflip_run, backflip_exit},
        {crossleg_enter, crossleg_run, crossleg_exit},
        {line_enter, line_run, line_exit},
        {jump_hurdles_enter, jump_hurdles_run, jump_hurdles_exit}, // ����
        {test1_enter, test1_run, test1_exit},
        {test2_enter, test2_run, test2_exit},
        {test3_enter, test3_run, test3_exit},
};

FSM_State *pCurrentState = &StateMap[Start]; // һ���洢��ǰ״̬��ָ��,������ʱ�л�

// ����״̬������������ʱ����һ��״̬�л�
void ChangeDogState(DOG_State x)
{
    if (pCurrentState->now_state != x)
    {
        pCurrentState->next_state = x;
        pCurrentState->execute_able = ENABLE;
    }
}

// ��ȡ��ǰ״̬
DOG_State GetNowDogState()
{
    return pCurrentState->now_state;
}

// ��ȡ��һ��״̬
DOG_State GetLastDogState()
{
    return pCurrentState->last_state;
}

// ��ȡ��һ�����л���״̬
DOG_State GetNextDogState()
{
    return pCurrentState->next_state;
}

// ����Ϊ���л�״̬,��execute_able��1ʱ����״̬�л�
void SetDogChangeAble(FunctionalState able)
{
    pCurrentState->change_able = able;
}

// ��ȡ�Ƿ���л�״̬
FunctionalState GetDogWhetherChangeAble()
{
    return pCurrentState->change_able;
}

// ��ȡ�Ƿ�Ҫ�л�״̬
FunctionalState GetDogWhetherExecuteAble()
{
    return pCurrentState->execute_able;
}

void DogFSM()
{
    if ((pCurrentState->change_able == ENABLE) && (pCurrentState->execute_able == ENABLE) && (pCurrentState->next_state != pCurrentState->now_state)) // �л�״̬
    {
        DOG_State now_state = pCurrentState->now_state;   // ���浱ǰ״̬
        DOG_State next_state = pCurrentState->next_state; // ������һ��״̬

        pCurrentState->exit(); // ִ�е�ǰ״̬���˳�����

        pCurrentState = &StateMap[pCurrentState->next_state]; // ���º���,��ʱ����״̬Ĭ��Ϊ0

        pCurrentState->last_state = now_state;  // ������һ��״̬
        pCurrentState->now_state = next_state;  // ���µ�ǰ״̬
        pCurrentState->next_state = next_state; // ������һ��״̬
        pCurrentState->change_able = DISABLE;   // ��ֹ�л�״̬
        pCurrentState->execute_able = DISABLE;

        pCurrentState->enter(); // ִ�е�ǰ״̬�Ľ��뺯��
        pCurrentState->run();   // ѭ��ִ��run
    }
    else
    {
        pCurrentState->run(); // ѭ��ִ��run
    }
}

__weak void start_enter() {}
__weak void start_exit() {}
__weak void start_run() {}

__weak void passive_enter() {}
__weak void passive_exit() {}
__weak void passive_run() {}

__weak void keep_enter() {}
__weak void keep_exit() {}
__weak void keep_run() {}

__weak void squat_enter() {}
__weak void squat_exit() {}
__weak void squat_run() {}

__weak void stand_enter() {}
__weak void stand_exit() {}
__weak void stand_run() {}

__weak void walk_enter() {}
__weak void walk_exit() {}
__weak void walk_run() {}

__weak void jump_enter() {}
__weak void jump_exit() {}
__weak void jump_run() {}

__weak void backflip_enter() {}
__weak void backflip_exit() {}
__weak void backflip_run() {}

__weak void crossleg_enter() {}
__weak void crossleg_exit() {}
__weak void crossleg_run() {}

__weak void line_enter() {}
__weak void line_exit() {}
__weak void line_run() {}

__weak void test1_enter() {}
__weak void test1_exit() {}
__weak void test1_run() {}

__weak void test2_enter() {}
__weak void test2_exit() {}
__weak void test2_run() {}

__weak void test3_enter() {}
__weak void test3_exit() {}
__weak void test3_run() {}

__weak void jump_hurdles_enter() {}
__weak void jump_hurdles_exit() {}
__weak void jump_hurdles_run() {}
