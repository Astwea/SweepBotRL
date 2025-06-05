#include "mpu6050.h"
#include "main.h"
#include "usart.h"
#include "stdio.h"
//#include "usart.h"   



//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//����,�������
extern I2C_HandleTypeDef hi2c1;
u8 MPU_Init(void)
{ 
	u8 res; 
	//MPU_IIC_Init();//��ʼ��IIC����
	HAL_I2C_Init(&hi2c1);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
  HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	//MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	HAL_Delay(2000);
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG); 
	printf("%x", res);
	if(res==0x70)//����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_READ,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(float *x,float *y,float *z)
{
    u8 buf[6],res;  
	short gx,gy,gz;
	res=MPU_Read_Len(MPU_READ,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		gx=((((u16)buf[0]<<8)|buf[1]));
		*x = (float)gx/16.4;
		gy=(((u16)buf[2]<<8)|buf[3]); 
		*y = (float)gy/16.4;
		gz=(((u16)buf[4]<<8)|buf[5]);
		*z = (float)gz/16.4;
	} 	
    return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(float *x,float *y,float *z)
{
  u8 buf[6],res;  
	short ax,ay,az;
	res=MPU_Read_Len(MPU_READ,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		ax=((u16)buf[0]<<8)|buf[1];
		*x = (float)ax/16384;
		ay=((u16)buf[2]<<8)|buf[3];  
		*y = (float)ay/16384;
		az=((u16)buf[4]<<8)|buf[5];
		*z = (float)az/16384;
	} 	
    return res;
}
//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
  extern I2C_HandleTypeDef hi2c1;
	if(HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff)==HAL_OK)
	{
		printf("\r\nWrite len current\r\n");
	}
	else{
	printf("\r\nerror\r\n");
	}
  
  HAL_Delay(1000);
  
  return 0;
}
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
  extern I2C_HandleTypeDef hi2c1;
	if (HAL_I2C_Mem_Read(&hi2c1,addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff) == HAL_OK)
	{
	}
	else{
	printf("\r\nRead error\r\n");
	}
  HAL_Delay(1);
  
  return 0;	
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
{ 
//  extern I2C_HandleTypeDef hi2c1;
  unsigned char W_Data=0;

  W_Data = data;
  if(HAL_I2C_Mem_Write(&hi2c1, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &W_Data, 1, 0xfff)==HAL_OK)
	{
		printf("\r\nWrite current\r\n");
		return 0;
	}
	else{
	printf("\r\nWrite error\r\n");
	}
  
  return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
uint8_t MPU_Read_Byte(uint8_t reg)
{
//  extern I2C_HandleTypeDef hi2c1;
  uint8_t R_Data[1]={0};
  
	HAL_I2C_Mem_Read(&hi2c1, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, R_Data, 1, 0xfff);
	
  
  return R_Data[0];		
}



uint8_t DMP_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
  extern I2C_HandleTypeDef hi2c1;
	HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
	//HAL_Delay(20);
	
	return 0;
}


uint8_t DMP_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
  extern I2C_HandleTypeDef hi2c1;
	HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
	//HAL_Delay(20);
	
	return 0;
}

