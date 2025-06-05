
#include "encoder.h"
#include "stdio.h"
Motor motor1;
Motor motor2;
uint8_t conut = 0;
void Motor_Init(void)
{
    HAL_TIM_Encoder_Start(&ENCODER_TIM_A, TIM_CHANNEL_ALL);      //������������ʱ��
    HAL_TIM_Encoder_Start(&ENCODER_TIM_B, TIM_CHANNEL_ALL);      //������������ʱ��
    __HAL_TIM_ENABLE_IT(&ENCODER_TIM_A,TIM_IT_CC1);           //������������ʱ�������ж�,���������
    __HAL_TIM_ENABLE_IT(&ENCODER_TIM_B,TIM_IT_CC1);           //������������ʱ�������ж�,���������
		__HAL_TIM_CLEAR_FLAG(&GAP_TIM,TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&GAP_TIM);                       //����100ms��ʱ���ж�
	   __HAL_TIM_ENABLE_IT(&GAP_TIM,TIM_IT_UPDATE); 
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_2);            //����PWM
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1);            //����PWM
    __HAL_TIM_SET_COUNTER(&ENCODER_TIM_A, 10000);                //��������ʱ����ʼֵ�趨Ϊ10000
    __HAL_TIM_SET_COUNTER(&ENCODER_TIM_B, 10000);  	//��������ʱ����ʼֵ�趨Ϊ10000

    motor1.lastCount = 0;                                   //�ṹ�����ݳ�ʼ��
    motor1.totalCount = 0;
    motor1.overflowNum = 0;                                  
    motor1.speed = 0;
    motor1.direct = 0;
		motor1.channel = TIM_CHANNEL_1;
		motor1.in1_pin = AIN1_Pin;
		motor1.in2_pin = AIN2_Pin;
	
    motor2.lastCount = 0;                                   //�ṹ�����ݳ�ʼ��
    motor2.totalCount = 0;
    motor2.overflowNum = 0;                                  
    motor2.speed = 0;
    motor2.direct = 0;
		motor2.channel = TIM_CHANNEL_2;
		motor2.in1_pin = BIN1_Pin;
		motor2.in2_pin = BIN2_Pin;
}


 // �����ԣ�50Hz������ֵ�����˲���Ч���ȽϺ�

float speed_Record_A[SPEED_RECORD_NUM]={0};
float speed_Record_B[SPEED_RECORD_NUM]={0};
/*
 * �����ٶȵ�ƽ���˲�
 * �����²��������ٶȣ�����ٶȵ����飬
 * �����˲�����ٶ�
 */
float Speed_Low_Filter(float new_Spe,float *speed_Record)
{
    float sum = 0.0f;
    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)//���������ݺ���һλ
    {
        speed_Record[i] = speed_Record[i-1];
        sum += speed_Record[i-1];
    }
    speed_Record[0] = new_Spe;//��һλ���µ�����
    sum += new_Spe;
    return sum/SPEED_RECORD_NUM;//���ؾ�ֵ
}

