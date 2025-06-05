/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f1xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//��ʱ���ص����������ڼ����ٶ�
{  
		if(htim->Instance==ENCODER_TIM_A.Instance)//���������붨ʱ������жϣ����ڷ����                   
    {      
        if(COUNTERNUM_A < 10000) motor1.overflowNum++;       //������������
        else if(COUNTERNUM_A >= 10000) motor1.overflowNum--; //������������
        __HAL_TIM_SetCounter(&ENCODER_TIM_A, 10000);      			//�����趨��ʼֵ

			__HAL_TIM_CLEAR_IT(&ENCODER_TIM_A, TIM_IT_UPDATE);
    }
		else if(htim->Instance==ENCODER_TIM_B.Instance)//���������붨ʱ������жϣ����ڷ����                   
    {      
        if(COUNTERNUM_B < 10000) motor2.overflowNum++;       //������������
        else if(COUNTERNUM_B >= 10000) motor2.overflowNum--; //������������
        __HAL_TIM_SetCounter(&ENCODER_TIM_B, 10000);      			//�����趨��ʼֵ

			__HAL_TIM_CLEAR_IT(&ENCODER_TIM_B, TIM_IT_UPDATE);
    }
    else if(htim->Instance==GAP_TIM.Instance)//�����ʱ���жϣ���ʱ������ٶ���
    {
        motor1.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER_TIM_A);//������ϼ�������ת��������ֵΪ0�����򷵻�ֵΪ1
        motor2.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&ENCODER_TIM_B);//������ϼ�������ת��������ֵΪ0�����򷵻�ֵΪ1
        motor1.totalCount = COUNTERNUM_A + motor1.overflowNum * RELOADVALUE_A;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        motor2.totalCount = COUNTERNUM_B + motor2.overflowNum * RELOADVALUE_B;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        conut ++;
        if(motor1.lastCount - motor1.totalCount > 19000) // �ڼ���ֵ���ʱ���з��������
        {
            motor1.overflowNum++;
            motor1.totalCount = COUNTERNUM_A + motor1.overflowNum * RELOADVALUE_A;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        }
        else if(motor1.totalCount - motor1.lastCount > 19000) // �ڼ���ֵ���ʱ���з��������
        {
            motor1.overflowNum--;
            motor1.totalCount = COUNTERNUM_A + motor1.overflowNum * RELOADVALUE_A;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        }
        if(motor2.lastCount - motor2.totalCount > 19000) // �ڼ���ֵ���ʱ���з��������
        {
            motor2.overflowNum++;
            motor2.totalCount = COUNTERNUM_B + motor2.overflowNum * RELOADVALUE_B;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        }
        else if(motor2.totalCount - motor2.lastCount > 19000) // �ڼ���ֵ���ʱ���з��������
        {
            motor2.overflowNum--;
            motor2.totalCount = COUNTERNUM_B + motor2.overflowNum * RELOADVALUE_B;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        }
        motor1.speed = (float)(motor1.totalCount - motor1.lastCount) / (4 * PULSE_PRE_ROUND*MOTOR_SPEED_RERATIO) * 100;//���ÿ�����ת,����4����Ϊ4��Ƶ
        /*******************����������˲�����************************/
        motor1.speed = -Speed_Low_Filter(motor1.speed,speed_Record_A);
        /**********************************************************/
        motor1.lastCount = motor1.totalCount; //��¼��һ�εļ���ֵ
				
				motor2.speed = (float)(motor2.totalCount - motor2.lastCount) / (4 * PULSE_PRE_ROUND*MOTOR_SPEED_RERATIO) * 100;//���ÿ�����ת,����4����Ϊ4��Ƶ
        /*******************����������˲�����************************/
        motor2.speed = Speed_Low_Filter(motor2.speed,speed_Record_B);
        /**********************************************************/
        motor2.lastCount = motor2.totalCount; //��¼��һ�εļ���ֵ
		}
}
/* USER CODE END 1 */
