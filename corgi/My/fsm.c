#include "fsm.h"
#include "stdint.h"
#include "A1_motor.h"
#include "dog.h"
#include "timer.h"

// 这里和同名.h里定义的enum是一一对应的,也就是StateMap[keep]对应的就是和keep状态有关的三个函数指针“keep_enter”“keep_run”“keep_exit”
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
        {jump_hurdles_enter, jump_hurdles_run, jump_hurdles_exit}, // 跨栏
        {test1_enter, test1_run, test1_exit},
        {test2_enter, test2_run, test2_exit},
        {test3_enter, test3_run, test3_exit},
};

FSM_State *pCurrentState = &StateMap[Start]; // 一个存储当前状态的指针,可以随时切换

// 设置状态并当条件允许时进行一次状态切换
void ChangeDogState(DOG_State x)
{
    if (pCurrentState->now_state != x)
    {
        pCurrentState->next_state = x;
        pCurrentState->execute_able = ENABLE;
    }
}

// 获取当前状态
DOG_State GetNowDogState()
{
    return pCurrentState->now_state;
}

// 获取上一个状态
DOG_State GetLastDogState()
{
    return pCurrentState->last_state;
}

// 获取下一个待切换的状态
DOG_State GetNextDogState()
{
    return pCurrentState->next_state;
}

// 设置为可切换状态,当execute_able置1时进行状态切换
void SetDogChangeAble(FunctionalState able)
{
    pCurrentState->change_able = able;
}

// 获取是否可切换状态
FunctionalState GetDogWhetherChangeAble()
{
    return pCurrentState->change_able;
}

// 获取是否要切换状态
FunctionalState GetDogWhetherExecuteAble()
{
    return pCurrentState->execute_able;
}

void DogFSM()
{
    if ((pCurrentState->change_able == ENABLE) && (pCurrentState->execute_able == ENABLE) && (pCurrentState->next_state != pCurrentState->now_state)) // 切换状态
    {
        DOG_State now_state = pCurrentState->now_state;   // 保存当前状态
        DOG_State next_state = pCurrentState->next_state; // 保存下一个状态

        pCurrentState->exit(); // 执行当前状态的退出函数

        pCurrentState = &StateMap[pCurrentState->next_state]; // 更新函数,此时所有状态默认为0

        pCurrentState->last_state = now_state;  // 更新上一个状态
        pCurrentState->now_state = next_state;  // 更新当前状态
        pCurrentState->next_state = next_state; // 更新下一个状态
        pCurrentState->change_able = DISABLE;   // 禁止切换状态
        pCurrentState->execute_able = DISABLE;

        pCurrentState->enter(); // 执行当前状态的进入函数
        pCurrentState->run();   // 循环执行run
    }
    else
    {
        pCurrentState->run(); // 循环执行run
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
