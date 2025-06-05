/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "stdio.h"
#include "oled.h"
#include "usart.h"
#include "encoder.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float pitch=0.0, roll = 0.0, yaw;
int load_sign = 0;
#define accur 0.015295 //18*3.3/4096 (3.3/4096����ADc�������ȣ�1:��Ϊ���ò���ת��һ���ܹ���ʾ���ʵ�λ��)
uint32_t adcValue;
float V;
/**********************************/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId pwmTaskHandle;
osThreadId mpuTaskHandle;
osThreadId oledTaskHandle;
osThreadId Usart_rosHandle;
osThreadId MOTO1Handle;
osThreadId MOTO2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void OledTask(void const * argument);
void usart_ros(void const * argument);
void MOTOR1(void const * argument);
void MOTOR2(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of pwmTask */
  osThreadDef(pwmTask, StartDefaultTask, osPriorityAboveNormal, 0, 128);
  pwmTaskHandle = osThreadCreate(osThread(pwmTask), NULL);

  /* definition and creation of mpuTask */
  osThreadDef(mpuTask, StartTask02, osPriorityHigh, 0, 128);
  mpuTaskHandle = osThreadCreate(osThread(mpuTask), NULL);

  /* definition and creation of oledTask */
  osThreadDef(oledTask, OledTask, osPriorityIdle, 0, 128);
  oledTaskHandle = osThreadCreate(osThread(oledTask), NULL);

  /* definition and creation of Usart_ros */
  osThreadDef(Usart_ros, usart_ros, osPriorityNormal, 0, 128);
  Usart_rosHandle = osThreadCreate(osThread(Usart_ros), NULL);

  /* definition and creation of MOTO1 */
  osThreadDef(MOTO1, MOTOR1, osPriorityNormal, 0, 128);
  MOTO1Handle = osThreadCreate(osThread(MOTO1), NULL);

  /* definition and creation of MOTO2 */
  osThreadDef(MOTO2, MOTOR2, osPriorityNormal, 0, 128);
  MOTO2Handle = osThreadCreate(osThread(MOTO2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
#include <math.h>



float wl=0.0, wr = 0.0;
#define PI 3.14159265358979323846
#define L 0.1   // ���峤�ȣ���λ���ף���������������;�ɺ���
#define W 0.2   // ���ּ�ࣨ��λ���ף�
#define D 0.06  // ����ֱ������λ���ף�

void calculateMotorSpeeds(float v1, float omega, float *RPM_left, float *RPM_right)
{
    const int reduction_ratio = 52;  // ������ٱ�

    if (v1 == 0 && omega == 0) {
        *RPM_left = *RPM_right = 0.0;
        return;
    }

    // ��ֱ���˶�
    if (omega == 0) {
        float rpm = (v1) / (PI * D) * reduction_ratio;
        *RPM_left = *RPM_right = rpm;
        return;
    }

    // ��ԭ����ת�������ٶȣ�
    if (v1 == 0) {
        float v_left = -omega * (W / 2.0f);  // �������ٶ�
        float v_right = omega * (W / 2.0f);  // �������ٶ�

        *RPM_left = (v_left) / (PI * D) * reduction_ratio;
        *RPM_right = (v_right) / (PI * D) * reduction_ratio;
        return;
    }

    // �����˶�
    float R = v1 / omega;                   // ת��뾶
    float R_left = R - (W / 2.0f);          // ���ְ뾶
    float R_right = R + (W / 2.0f);         // ���ְ뾶

    float v_left = v1 * (R_left / R);       // �������ٶ�
    float v_right = v1 * (R_right / R);     // �������ٶ�

    *RPM_left = (v_left) / (PI * D) * reduction_ratio;
    *RPM_right = (v_right) / (PI * D) * reduction_ratio;
}

#define kp1 10.0
#define ki1 0.0
#define kd1 0.0
#define kp2 10.0
#define ki2 0.0
#define kd2 0.0
#define dt 0.01
float i1 = 0.0;      // ������
float le1 = 0.0;    // ��һ�����
float i2 = 0.0;      // ������
float le2 = 0.0;    // ��һ�����
int err_wheel1 = 0;
int err_wheel2 = 0;
int max_pwm = 0;
int PID_Speed_1(float speed, float target_speed, float *i, float *le)
{
	float err = target_speed - speed;
	*i += err * dt;
	float d = (err - *le)/dt;
	int output = kp1*err + ki1*(*i) + kd1 * d;
	if (output > max_pwm) output = max_pwm;
	if (output < -max_pwm) output = -max_pwm;
	return output;
	
}
int PID_Speed_2(float speed, float target_speed, float *i, float *le)
{
	float err = target_speed - speed;
	*i += err * dt;
	float d = (err - *le)/dt;
	int output = kp1*err + ki1*(*i) + kd1 * d;
	if (output > max_pwm) output = max_pwm;
	if (output < -max_pwm) output = -max_pwm;
	return output;
	
}
float v = 0.0, w = 0.0;
int slogn = 0;
int xi =0;
int sao=0;
int no_Ground = 0;
int mpu_state = 1;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

	
  while(1)
  {

		if(flag==1)
		{
				printf(Buffer);
				for(int i = 0; i < 20
			; i++)
				{

						if (Buffer[i] == '$')  // �ҵ� '$'
						{
								int num_items = sscanf(&Buffer[i+1], "%f %f %d %d", &v, &w,&xi, &sao);
							  printf("\r\n%f %f %d %d\r\n", v, w,xi, sao);
								if(v > 0.08)
								{
									v = 0.08;
								}
								else if(v < -0.08)
								{
									v = -0.08;
								}
								if(w > 0.2)
								{
									w = 0.2;
								}
								else if(w < -0.2)
								{
									w = -0.2;
								}
								v = -v;
								w = -w;
						}
						if (Buffer[i] == 'R')
						{
							printf("\r\n ReBoot MPU6050\r\n");
							mpu_state = 1;							
						}
				}
				if(xi == 1)
				{
					 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
					 printf("Set Xi Over\r\n");
				}
				else
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
				}
				if(sao == 1)
				{
					 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
					 printf("Set Sao Over\r\n");
				}
				else
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
				}
				calculateMotorSpeeds(v, w, &wl, &wr);
		   	flag=0;
				memset(Buffer, 0, sizeof(Buffer));	
		}

//		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) 
//		{
//			no_Ground = 0;
//		} 
//		else 
//		{
//			no_Ground = 1;
//		}

//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
//		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
		//value --;
		osDelay(50);
		
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the mpuTask thread.
* @param argument: Not used
* @retval None
*/
float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
float accel_bias_x = 0, accel_bias_y = 0;
float sum_ax = 0, sum_ay = 0;
float g_to_mps2 = 9.80665f;
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	for(;;)
  {
		if(mpu_state){
			MPU_Init();
			osDelay(200);
			while(mpu_dmp_init())                        
			{
			}
			load_sign = 1;
		//	int conut = 0;
		//  /* Infinite loop */
			int bias_sample_count = 100; // �ɼ�100��
			float sum_gx = 0, sum_gy = 0, sum_gz = 0;
			for(int i = 0; i < bias_sample_count; i++)
			{
					MPU_Get_Gyroscope(&gx, &gy, &gz);
				  MPU_Get_Accelerometer(&ax, &ay, &az);
				
				  sum_ax += ax;
					sum_ay += ay;
				
					sum_gx += gx;
					sum_gy += gy;
					sum_gz += gz;

					osDelay(10); // ÿ��10ms����ֹˢ̫��
			}
			accel_bias_x = sum_ax / bias_sample_count;
			accel_bias_y = sum_ay / bias_sample_count;
			
			gyro_bias_x = sum_gx / bias_sample_count;
			gyro_bias_y = sum_gy / bias_sample_count;
			gyro_bias_z = sum_gz / bias_sample_count;
			mpu_state = 0;
		}

			while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0);
 		  MPU_Get_Accelerometer(&ax,&ay,&az);
			MPU_Get_Gyroscope(&gx,&gy,&gz);
		
		  ax = (ax - accel_bias_x)*g_to_mps2;
			ay = (ay - accel_bias_y)*g_to_mps2;
		
			gx = gx - gyro_bias_x;
			gy = gy - gyro_bias_y;
			gz = gz - gyro_bias_z;
			osDelay(30);

  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_OledTask */
/**
* @brief Function implementing the oledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OledTask */
void OledTask(void const * argument)
{
  /* USER CODE BEGIN OledTask */
  /* Infinite loop */
	char str[20];
	osDelay(1000);
	OLED_Init();
	OLED_Refresh();
  for(;;)
  {
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      // ��ȡ ADC ת��ֵ
      adcValue = HAL_ADC_GetValue(&hadc1);
			V = (adcValue * 3.3 / 4095.0) * 5.87;
			//printf("%.3f, %d\r\n",motor1.speed, (int)v);
		}
		if(load_sign != 0)
		{
			sprintf(str, "%.3f", V);
			OLED_ShowString(0, 0,  str, 12);
			OLED_ShowString(0, 20, "MPU-> Pitch:", 12);
			sprintf(str, "%.3f", pitch);
			OLED_ShowString(80, 20,  str, 12);
			OLED_ShowString(30, 30," Roll:", 12);
			sprintf(str, "%.3f", roll);
			OLED_ShowString(80, 30, str,12);
			OLED_ShowString(30, 40," Yaw: :", 12);
			sprintf(str, "%.3f", yaw);
			OLED_ShowString(80, 40, str,12);
			OLED_Refresh();
		}
 		else
		{
			sprintf(str, "%.4f", V);

			OLED_ShowString(0, 0,  str, 12);
			OLED_ShowString(0, 30, "----MPU_Loading----", 12);
			OLED_Refresh();
		} 
		max_pwm = 7.4/V * 3199;
		osDelay(200);
  }
  /* USER CODE END OledTask */
}

/* USER CODE BEGIN Header_usart_ros */
/**
* @brief Function implementing the Usart_ros thread.
* @param argument: Not used
* @retval None
*/
uint8_t txBuffer[50];
uint16_t calculate_crc(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;  // ��ʼ��CRCֵ
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint8_t tx_Buffer[64];                // ��������
void send_data(void)
{		
    if (uart_tx_ready)  // ���UART����
    {
        int index = 0;
				
        tx_Buffer[index++] = '$';
        memcpy(&tx_Buffer[index], &motor1.speed, sizeof(motor1.speed)); index += sizeof(motor1.speed);
        memcpy(&tx_Buffer[index], &motor2.speed, sizeof(motor2.speed)); index += sizeof(motor2.speed);
        memcpy(&tx_Buffer[index], &pitch, sizeof(pitch)); index += sizeof(pitch);
        memcpy(&tx_Buffer[index], &roll, sizeof(roll)); index += sizeof(roll);
        memcpy(&tx_Buffer[index], &yaw, sizeof(yaw)); index += sizeof(yaw);
        memcpy(&tx_Buffer[index], &ax, sizeof(ax)); index += sizeof(ax);
        memcpy(&tx_Buffer[index], &ay, sizeof(ay)); index += sizeof(ay);
        memcpy(&tx_Buffer[index], &az, sizeof(az)); index += sizeof(az);
        memcpy(&tx_Buffer[index], &gx, sizeof(gx)); index += sizeof(gx);
        memcpy(&tx_Buffer[index], &gy, sizeof(gy)); index += sizeof(gy);
        memcpy(&tx_Buffer[index], &gz, sizeof(gz)); index += sizeof(gz);

        uint16_t crc = calculate_crc(tx_Buffer + 1, index - 1);
        tx_Buffer[index++] = (uint8_t)(crc >> 8);
        tx_Buffer[index++] = (uint8_t)(crc & 0xFF);

        tx_Buffer[index++] = '#';

        uart_tx_ready = 0;  // ��־λ����ʾ���ڷ���
        HAL_UART_Transmit_DMA(&huart1, tx_Buffer, index);  // ����DMA����
    }
}

/* USER CODE END Header_usart_ros */
void usart_ros(void const * argument)
{
  /* USER CODE BEGIN usart_ros */
  /* Infinite loop */
  for(;;)
  {
//	  send_data();
		printf("%.3f,%.3f,%.3f,%.3f,%d,%d\r\n", wl,wr,motor1.speed,motor2.speed,no_Ground, max_pwm);
    osDelay(20);
  }
  /* USER CODE END usart_ros */
}

/* USER CODE BEGIN Header_MOTOR1 */
/**
* @brief Function implementing the MOTO1 thread.
* @param argument: Not used
* @retval None
*/
void Motor_Control(int speed, Motor motor) {
	if(no_Ground == 0)
	{
    if (speed > 0) {
        // ������ת
        HAL_GPIO_WritePin(GPIOB, motor.in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, motor.in2_pin, GPIO_PIN_RESET);
    } else if (speed < 0) {
        // ���÷�ת
        HAL_GPIO_WritePin(GPIOB, motor.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, motor.in2_pin, GPIO_PIN_SET);
        speed = -speed;  // תΪ��ֵ�������� PWM ռ�ձ�
    } else {
        // ֹͣ
        HAL_GPIO_WritePin(GPIOB, motor.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, motor.in2_pin, GPIO_PIN_RESET);
    }
	}
	else
	{
		    HAL_GPIO_WritePin(GPIOB, motor.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, motor.in2_pin, GPIO_PIN_RESET);
	}

    // Debug����ӡ���Ƶ�ƽ


    // ���� PWM ռ�ձȣ�����ת��
    __HAL_TIM_SET_COMPARE(&htim2, motor.channel, speed);
}

/* USER CODE END Header_MOTOR1 */
float symmetry_k = 0.05;  // �������ӣ����������
void MOTOR1(void const * argument)
{
  /* USER CODE BEGIN MOTOR1 */
  /* Infinite loop */
  for(;;)
  {
		float speed_diff = motor1.speed - motor2.speed;  // �� - ��
		if(no_Ground == 0){
			err_wheel1 += PID_Speed_1(motor1.speed, wl, &i1, &le1);
		}
		if(wl == 0)
		{
			err_wheel1 = 0;
		}
		err_wheel1 -= symmetry_k * speed_diff;
		Motor_Control(err_wheel1, motor1);
		
    osDelay(20);
  }
  /* USER CODE END MOTOR1 */
}

/* USER CODE BEGIN Header_MOTOR2 */
/**
* @brief Function implementing the MOTO2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MOTOR2 */
void MOTOR2(void const * argument)
{
  /* USER CODE BEGIN MOTOR2 */
  /* Infinite loop */
  for(;;)
  {
		float speed_diff = motor1.speed - motor2.speed;
		if(no_Ground == 0){
			err_wheel2 += PID_Speed_2(motor2.speed, wr, &i2, &le2);
		}
		if(wr == 0)
		{
			err_wheel2 = 0;
		}
		err_wheel2 += symmetry_k * speed_diff;
  	Motor_Control(err_wheel2, motor2);
		
    osDelay(20);
  }
  /* USER CODE END MOTOR2 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

