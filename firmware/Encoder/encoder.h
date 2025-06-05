
#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "stm32f1xx.h"
#include "tim.h"
//电机1的编码器输入引脚
#define MOTO1_ENCODER1_PORT GPIOB
#define MOTO1_ENCODER1_PIN  GPIO_PIN_6
#define MOTO1_ENCODER2_PORT GPIOB
#define MOTO1_ENCODER2_PIN  GPIO_PIN_7
#define SPEED_RECORD_NUM 20
//电机2的编码器输入引脚
#define MOTO2_ENCODER1_PORT GPIOB
#define MOTO2_ENCODER1_PIN  GPIO_PIN_4
#define MOTO2_ENCODER2_PORT GPIOB
#define MOTO2_ENCODER2_PIN  GPIO_PIN_5
#define SPEED_RECORD_NUM 20
//定时器号
#define PWM_TIM htim2
#define ENCODER_TIM_A htim4
#define ENCODER_TIM_B htim3
#define GAP_TIM     htim1

#define AIN1_Pin GPIO_PIN_12
#define AIN2_Pin GPIO_PIN_13
#define BIN1_Pin GPIO_PIN_14
#define BIN2_Pin GPIO_PIN_15

#define MOTOR_SPEED_RERATIO 20u    //电机减速比
#define PULSE_PRE_ROUND 13 //一圈多少个脉冲
#define RADIUS_OF_TYRE 29 //轮胎半径，单位毫米
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14
#define RELOADVALUE_A __HAL_TIM_GetAutoreload(&ENCODER_TIM_A)    //获取自动装载值,本例中为20000
#define COUNTERNUM_A __HAL_TIM_GetCounter(&ENCODER_TIM_A)        //获取编码器定时器中的计数值
#define RELOADVALUE_B __HAL_TIM_GetAutoreload(&ENCODER_TIM_B)    //获取自动装载值,本例中为20000
#define COUNTERNUM_B __HAL_TIM_GetCounter(&ENCODER_TIM_B)        //获取编码器定时器中的计数值
extern float speed_Record_A[SPEED_RECORD_NUM];
extern float speed_Record_B[SPEED_RECORD_NUM];
void Motor_Init(void);
float Speed_Low_Filter(float new_Spe,float *speed_Record);
typedef struct _Motor
{
    int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
    float speed;         //电机转速
    uint8_t direct;      //旋转方向
		uint32_t channel;
		uint16_t in1_pin;
		uint16_t in2_pin;
}Motor;
extern Motor motor1;
extern Motor motor2;
extern uint8_t conut;
#endif
