
#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "stm32f1xx.h"
#include "tim.h"
//���1�ı�������������
#define MOTO1_ENCODER1_PORT GPIOB
#define MOTO1_ENCODER1_PIN  GPIO_PIN_6
#define MOTO1_ENCODER2_PORT GPIOB
#define MOTO1_ENCODER2_PIN  GPIO_PIN_7
#define SPEED_RECORD_NUM 20
//���2�ı�������������
#define MOTO2_ENCODER1_PORT GPIOB
#define MOTO2_ENCODER1_PIN  GPIO_PIN_4
#define MOTO2_ENCODER2_PORT GPIOB
#define MOTO2_ENCODER2_PIN  GPIO_PIN_5
#define SPEED_RECORD_NUM 20
//��ʱ����
#define PWM_TIM htim2
#define ENCODER_TIM_A htim4
#define ENCODER_TIM_B htim3
#define GAP_TIM     htim1

#define AIN1_Pin GPIO_PIN_12
#define AIN2_Pin GPIO_PIN_13
#define BIN1_Pin GPIO_PIN_14
#define BIN2_Pin GPIO_PIN_15

#define MOTOR_SPEED_RERATIO 20u    //������ٱ�
#define PULSE_PRE_ROUND 13 //һȦ���ٸ�����
#define RADIUS_OF_TYRE 29 //��̥�뾶����λ����
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14
#define RELOADVALUE_A __HAL_TIM_GetAutoreload(&ENCODER_TIM_A)    //��ȡ�Զ�װ��ֵ,������Ϊ20000
#define COUNTERNUM_A __HAL_TIM_GetCounter(&ENCODER_TIM_A)        //��ȡ��������ʱ���еļ���ֵ
#define RELOADVALUE_B __HAL_TIM_GetAutoreload(&ENCODER_TIM_B)    //��ȡ�Զ�װ��ֵ,������Ϊ20000
#define COUNTERNUM_B __HAL_TIM_GetCounter(&ENCODER_TIM_B)        //��ȡ��������ʱ���еļ���ֵ
extern float speed_Record_A[SPEED_RECORD_NUM];
extern float speed_Record_B[SPEED_RECORD_NUM];
void Motor_Init(void);
float Speed_Low_Filter(float new_Spe,float *speed_Record);
typedef struct _Motor
{
    int32_t lastCount;   //��һ�μ���ֵ
    int32_t totalCount;  //�ܼ���ֵ
    int16_t overflowNum; //�������
    float speed;         //���ת��
    uint8_t direct;      //��ת����
		uint32_t channel;
		uint16_t in1_pin;
		uint16_t in2_pin;
}Motor;
extern Motor motor1;
extern Motor motor2;
extern uint8_t conut;
#endif
