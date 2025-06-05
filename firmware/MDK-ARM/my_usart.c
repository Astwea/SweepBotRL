#include "usart.h"
#include "FreeRTOS.h"
uint8_t Receive_Pocket_Max_Len = 50;
uint8_t Receive_Mode = 0;//����ģʽ�����ֽڻ����ݰ�
uint8_t Receive_State;//״̬������
uint8_t Receive_Byte[1],Receive_ITFlag;//���յ��ֽ����ݣ����ձ�־λ//!!!���ﵥ�ֽڽ��ջ���һ��Ҫ��������ʽ!!!
uint8_t Receive_PocketData[Receive_Pocket_Max_Len] = {0};//�������ݰ�����
uint16_t Receive_Pocket_Index = 0;//����ָ��
 
extern QueueHandle_t xQueueHandle_Usart3;//���о��
 
void Serial_ChangeMode(Serial_Mode Mode)
{
	Receive_Mode = (uint8_t)Mode;
}
 
void Serial_SendByte(uint8_t Byte)
{
	//���������������ʱ�����ã��������ݻᴫ�����ԭ���ݲ���
	//HAL_UART_Transmit_IT(&huart3,&Byte,1);
	HAL_UART_Transmit(&huart1,&Byte,1,HAL_MAX_DELAY);
}
 
void Serial_SendString(uint8_t *String)
{
	HAL_UART_Transmit(&huart1,String,strlen((const char*)String),HAL_MAX_DELAY);
}
 
void Serial_SendArray(uint8_t* ArrayData,uint16_t Length)
{
	HAL_UART_Transmit(&huart1,ArrayData,Length,HAL_MAX_DELAY);
}
 
void Serical_SendPocket(uint8_t* PocketData)
{
	Serial_SendByte(0xFD);
	Serial_SendArray(PocketData,Receive_Pocket_Index);
	Serial_SendByte(0xFE);
	Serial_SendByte(0xFF);
}
//���յ��ֽں����ݰ���־λ
uint8_t Serial_GetReceiveFlag(void)
{
	if(Receive_ITFlag == 1)
	{
		Receive_ITFlag = 0;
		return 1;
	}
	return 0;
}
 
//���ֽ�����
uint8_t Serial_ReceiveByte(void)
{
	return Receive_Byte[0];
}
//���ݰ�����
void Serial_ReceivePocket(uint8_t *PocketArray)
{
	uint8_t i = 0;
	Receive_GetDataState = 0;
	for (i = 0; Receive_PocketData[i] != '\0'; i ++)
	{
		PocketArray[i] = Receive_PocketData[i];
	}
}
//���ݰ�����
uint16_t Serial_GetPocketLength(void)
{
	return Receive_Pocket_Index;
}