
#include "encoder.h"
#include "stdio.h"
Motor motor1;
Motor motor2;
uint8_t conut = 0;
void Motor_Init(void)
{
    HAL_TIM_Encoder_Start(&ENCODER_TIM_A, TIM_CHANNEL_ALL);      //开启编码器定时器
    HAL_TIM_Encoder_Start(&ENCODER_TIM_B, TIM_CHANNEL_ALL);      //开启编码器定时器
    __HAL_TIM_ENABLE_IT(&ENCODER_TIM_A,TIM_IT_CC1);           //开启编码器定时器更新中断,防溢出处理
    __HAL_TIM_ENABLE_IT(&ENCODER_TIM_B,TIM_IT_CC1);           //开启编码器定时器更新中断,防溢出处理
		__HAL_TIM_CLEAR_FLAG(&GAP_TIM,TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&GAP_TIM);                       //开启100ms定时器中断
	   __HAL_TIM_ENABLE_IT(&GAP_TIM,TIM_IT_UPDATE); 
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_2);            //开启PWM
    HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1);            //开启PWM
    __HAL_TIM_SET_COUNTER(&ENCODER_TIM_A, 10000);                //编码器定时器初始值设定为10000
    __HAL_TIM_SET_COUNTER(&ENCODER_TIM_B, 10000);  	//编码器定时器初始值设定为10000

    motor1.lastCount = 0;                                   //结构体内容初始化
    motor1.totalCount = 0;
    motor1.overflowNum = 0;                                  
    motor1.speed = 0;
    motor1.direct = 0;
		motor1.channel = TIM_CHANNEL_1;
		motor1.in1_pin = AIN1_Pin;
		motor1.in2_pin = AIN2_Pin;
	
    motor2.lastCount = 0;                                   //结构体内容初始化
    motor2.totalCount = 0;
    motor2.overflowNum = 0;                                  
    motor2.speed = 0;
    motor2.direct = 0;
		motor2.channel = TIM_CHANNEL_2;
		motor2.in1_pin = BIN1_Pin;
		motor2.in2_pin = BIN2_Pin;
}


 // 经测试，50Hz个采样值进行滤波的效果比较好

float speed_Record_A[SPEED_RECORD_NUM]={0};
float speed_Record_B[SPEED_RECORD_NUM]={0};
/*
 * 进行速度的平均滤波
 * 输入新采样到的速度，存放速度的数组，
 * 返回滤波后的速度
 */
float Speed_Low_Filter(float new_Spe,float *speed_Record)
{
    float sum = 0.0f;
    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)//将现有数据后移一位
    {
        speed_Record[i] = speed_Record[i-1];
        sum += speed_Record[i-1];
    }
    speed_Record[0] = new_Spe;//第一位是新的数据
    sum += new_Spe;
    return sum/SPEED_RECORD_NUM;//返回均值
}

