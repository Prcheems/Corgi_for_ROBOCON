#ifndef BACKFLIP_H
#define BACKFLIP_H

#include "dog.h"
#include "stdbool.h"
#include "action.h"
#include "imu.h"

typedef struct
{
    float Prep_Rho[4];         // ����ʱ�ؽڵ������˵ľ���
    float Prep_Theta[4];       // ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
    float Prep_Time[4];        // ����ʱ���ȶ����õ�ʱ��
    float Prep_Buffer_Time[4]; // ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
                               // ����ʱ�õ���ʱ����Prep_Time+Prep_Buffer_Time

    float Backflip_1_Rho[4];     		  // ��һ������ʱ�ؽڵ������˵ľ���
    float Backflip_1_Theta[4];   		  // ��һ������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
    float Backflip_1_Time[4];      // ��һ������ʱ���ȶ����õ�ʱ��

    float Shift_Rho[4];            // ��һ������������ڿ��аڶ����������һ����ʱ�ؽڵ������˵ľ���
    float Shift_Theta[4];          // ��һ������������ڿ��аڶ����������һ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
    float Shift_Time[4];           // ��һ������������ڿ��аڶ����������һ�����õ�ʱ��

    float Backflip_2_Rho[4];   // �ڶ�������ʱ�ؽڵ������˵ľ���
    float Backflip_2_Theta[4]; // �ڶ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
    float Backflip_2_Time[4];  // �ڶ�������ʱ���ȶ����õ�ʱ��

    float Load_Rho[4];          // ��½ʱ�ؽڵ������˵ľ���
    float Load_Theta[4];        // ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
    float Swing_in_Air_Time[4]; // �ڶ���������ɺ���ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������
    float Land_Buffer_Time[4];  // ��ػ���ʱ��

    float Std_Prep_Theta[4];       // ��׼�Ƕ�,��01��Ϊ��׼�ĽǶ�,���������ֱ��ͷ���������õ�
    float Std_Backflip_1_Theta[4]; // ��׼�Ƕ�,��01��Ϊ��׼�ĽǶ�,���������ֱ��ͷ���������õ�
    float Std_Backflip_2_Theta[4]; // ��׼�Ƕ�,��01��Ϊ��׼�ĽǶ�,���������ֱ��ͷ���������õ�
    float Std_Shift_Theta[4];      // ��׼�Ƕ�,��01��Ϊ��׼�ĽǶ�,���������ֱ��ͷ���������õ�
    float Std_Load_Theta[4];       // ��׼�Ƕ�,��01��Ϊ��׼�ĽǶ�,���������ֱ��ͷ���������õ�

} BackflipParam;

typedef struct
{
    PD VMC_Swing[4];   // �ڶ�����
    PD VMC_Cushion[4]; // ��ػ������
} Backflip_VMC_Param;

typedef enum
{
    First_Flip,		// ��һ����Ծ,��һ��ǰ�շ�
		Second_Flip,	// �ڶ�����Ծ,�ڶ���ǰ�շ�
} BackflipType;

bool Backflip_s(uint8_t legid, BackflipParam *Backflip_Param, action_param *Action_Param, Euler *Euler_Angle, float t);
extern BackflipParam Backflip_Param[2];
extern BackflipType Backfliptype;

#endif

