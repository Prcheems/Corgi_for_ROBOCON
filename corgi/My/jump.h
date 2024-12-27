#ifndef JUMP_H
#define JUMP_H

#include "dog.h"
#include "stdbool.h"
#include "action.h"
#include "imu.h"
#include "A1_motor.h"

typedef struct
{
    float Prep_Rho[4];         // ����ʱ�ؽڵ������˵ľ���
    float Prep_Theta[4];       // ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
    float Prep_Time[4];        // ����ʱ���ȶ����õ�ʱ��
    float Prep_Buffer_Time[4]; // ����ʱ������ɺ�Ļ���ʱ��,�����Ժ�ֱ��������ΪҪ��֤������ʱ�仺�嵽�ȶ�״̬
                               // ����ʱ�õ���ʱ����Prep_Time+Prep_Buffer_Time

    float Jump_Rho[4];         // ����ʱ�ؽڵ������˵ľ���
    float Jump_Theta[4];       // ����ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
    float Jump_Time[4];        // ����ʱ���ȶ����õ�ʱ��
    float Jump_Buffer_Time[4]; // ����ʱ������ɺ�Ļ���ʱ��
                               // �����õ���ʱ����Jump_Time+Jump_Buffer_Time

    float Tuck_Rho[4];          // �������ڿ�������ʱ�ؽڵ������˵ľ���
    float Tuck_Theta[4];        // �������ڿ�������ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
    float Tuck_Time[4];         // �������ڿ��������õ�ʱ��
    float Tuck_Buffer_Time[4];  // �������ڿ���������ɺ�Ļ���ʱ��,�����Ժ�ȴ����ǰ����
    float Swing_in_Air_Time[4]; // ������ɺ��ȵĻذ�ʱ��,�Ȱڻ�ȥ�Ժ�������

    float Load_Rho[4];         // ��½ʱ�ؽڵ������˵ľ���
    float Load_Theta[4];       // ��½ʱ�ؽڵ������˵�������X������ļн�0,��ʱ��Ϊ��
    float Land_Buffer_Time[4]; // ��ػ���ʱ��

    float Std_Prep_Theta; // ��׼�Ƕ�,��01��Ϊ��׼�ĽǶ�,���������ֱ��ͷ���������õ�
    float Std_Jump_Theta; // ��׼�Ƕ�,��01��Ϊ��׼�ĽǶ�,���������ֱ��ͷ���������õ�
    float Std_Tuck_Theta; // ��׼�Ƕ�,��01��Ϊ��׼�ĽǶ�,���������ֱ��ͷ���������õ�
    float Std_Load_Theta; // ��׼�Ƕ�,��01��Ϊ��׼�ĽǶ�,���������ֱ��ͷ���������õ�

} JumpParam;

typedef struct
{
    PD VMC_Swing[4];   // �ڶ�����
    PD VMC_Cushion[4]; // ��ػ������
		float Preset_Kp_x[4];
		float Preset_Kp_z[4];
		float Boost_x;
		float Boost_z;
} Jump_VMC_Param;

typedef enum
{
	  First_Step = 0,   // 
    Second_Step,  		// 
    Third_Step,  			// 
    Bridge,
		Bridge2,	// 
    GroundForward,    // ƽ����ǰ��Ծ
    GroundBack,       // ƽ�������Ծ
    GroundLeft,       // ԭ����ת��Ծ
    GroundRight,      // ԭ����ת��Ծ
    GroundUpToStep,   // ƽ�ص�̨��

    StepDownToGround, // ��̨�׵�ƽ��
    ClimbUp,          // ����
    ClimbDown,        // ����
    JumpTest1,        // ������
    JumpTest2,        // ǰ�ȱ� (���ڿ���)
    JumpTest3,        // ���ȱ� (���ڿ���)
} JumpType;

bool Jump_s(uint8_t legid, JumpParam *Jump_Param, action_param *Action_Param, Euler *Euler_Angle, float t);
extern JumpParam Jump_Param[15];
extern Jump_VMC_Param Jump_VMC;
#endif
