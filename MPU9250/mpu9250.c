/*
 * mpu9250.c
 *
 *  Created on: 2021年10月15日
 *      Author: chen
 */

#include "mpu9250.h"
#include "stdint.h"
#include "gd32vf103.h"
#include "myi2c.h"
#include <stdio.h>
float acc_[3];
float gyro_[3];
short mag_[3];

/********************************************************************************************/
/* 							PART1: MPU9250 初始化配置										*/
/********************************************************************************************/

uint8_t MPU9250_Init(void)
{
	uint8_t res = 0;
	I2C_WriteByte(MPU6500_ADDR, MPU9250_PWR_MGMT1_REG, 0x80);
	delay_1ms(100);
	I2C_WriteByte(MPU6500_ADDR, MPU9250_PWR_MGMT2_REG, 0x00);
	MPU9250_SetGyroFsr(MPU9250_GYRO_FSR);
	MPU9250_SetAccelFsr(MPU9250_ACCEL_FSR_2g);
	MPU9250_SetRate(MPU9250_RATE);
	I2C_WriteByte(MPU6500_ADDR, MPU9250_INT_EN_REG, 0x00);
	I2C_WriteByte(MPU6500_ADDR, MPU9250_USER_CTRL_REG, 0x00);
	I2C_WriteByte(MPU6500_ADDR, MPU9250_FIFO_EN_REG, 0x00);
	I2C_WriteByte(MPU6500_ADDR, MPU9250_INTBP_CFG_REG, 0x82);
	res = I2C_ReadByte(MPU6500_ADDR, MPU9250_DEVICE_ID_REG);
	while (res!=MPU6500_ID) {
		delay_1ms(1000);
		res = I2C_ReadByte(MPU6500_ADDR, MPU9250_DEVICE_ID_REG);
		printf("res: %d\r\n",res);
	}
	if (res == MPU6500_ID)
	{
		I2C_WriteByte(MPU6500_ADDR, MPU9250_PWR_MGMT1_REG, 0x01);
		I2C_WriteByte(MPU6500_ADDR, MPU9250_PWR_MGMT2_REG, 0x00);
	}
	else
	{
		printf("MPU9250 Init failed 1, res = %X\r\n", res);
		return 1;
	}
	res = I2C_ReadByte(AK8963_ADDR, MPU9250_MAG_WIA);
	if (res == AK8963_ID)
	{
		I2C_WriteByte(AK8963_ADDR, MPU9250_MAG_CNTL1, 0x11);		// 连续测量模式
		printf("MPU9250 Init success.\r\n");
	}
	else
	{

		printf("MPU9250 Init failed 2, res = %X\r\n", res);
		return 1;
	}
	return 0;
}


/* 设置MPU9250陀螺仪传感器满量程范围					*/
/* fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps	*/
/* 返回值:0,设置成功									*/
/*    其他,设置失败 									*/

void MPU9250_SetGyroFsr(uint8_t fsr)
{
	I2C_WriteByte(MPU6500_ADDR, MPU9250_GYRO_CFG_REG, fsr<<3);	//设置陀螺仪满量程范围
}

/* 设置MPU92506050加速度传感器满量程范围	*/
/* fsr:0,±2g;1,±4g;2,±8g;3,±16g		*/
void MPU9250_SetAccelFsr(uint8_t fsr)
{
	I2C_WriteByte(MPU6500_ADDR, MPU9250_ACCEL_CFG_REG, fsr<<3);	//设置加速度传感器满量程范围
}

/* 设置MPU92506050的数字低通滤波器		*/
/* lpf:数字低通滤波频率(Hz)				*/
void  MPU9250_SetLPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)
		data=1;
	else if(lpf>=98)
		data=2;
	else if(lpf>=42)
		data=3;
	else if(lpf>=20)
		data=4;
	else if(lpf>=10)
		data=5;
	else
		data=6;
	I2C_WriteByte(MPU6500_ADDR, MPU9250_CFG_REG, data);			//设置数字低通滤波器
}

/* 设置MPU9250的采样率(假定Fs=1KHz)		*/
/* rate:4~1000(Hz)						*/

void MPU9250_SetRate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)
		rate=1000;
	if(rate<4)
		rate=4;
	data = 1000 / rate - 1;
	I2C_WriteByte(MPU6500_ADDR, MPU9250_SAMPLE_RATE_REG, data);	//设置数字低通滤波器
 	MPU9250_SetLPF(rate / 2);											//自动设置LPF为采样率的一半
}

/********************************************************************************************/
/* 							PART2: 校准函数													*/
/********************************************************************************************/

/* 校准加速度 			*/
/* 采用简单的六面校准 	*/

void CaliAccelData(float acc[3])
{
	static const float acc_offset[3] = {0, 0, 0};		// 多次测量加速度计后估计的偏移值
	float acc_norm;											// 加速度模值
	const float g = 9.8;									// 重力加速度

	acc[0] = -acc[0] - acc_offset[0];						// 去除偏移，此处添加负号使得与机体坐标系一致
	acc[1] = acc[1] - acc_offset[1];
	acc[2] = acc[2] - acc_offset[2];

	acc_norm = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
	acc[0] = acc[0] / acc_norm * g;							// 归一化到 [-g, g]
	acc[1] = acc[1] / acc_norm * g;
	acc[2] = acc[2] / acc_norm * g;
}


/* 校准陀螺仪 	*/
/* 去除零偏		*/
void CaliGyroData(float gyro[3])
{
	static const float gyro_offset[3] = {0,  0,  0};	// 测量陀螺仪数据后估计的陀螺仪偏移值

	gyro[0] = -(gyro[0] - gyro_offset[0]);
	gyro[1] = -(gyro[1] - gyro_offset[1]);
	gyro[2] = -(gyro[2] - gyro_offset[2]);
}

/* 校准磁力计	*/
/* 校准零偏		*/
void CaliMagData(short mag[3])
{
	static const short mag_offset[3] = {0,  0,  0};		// 校准磁力计使用的偏移，多次测量计算而得

	mag[0] -= mag_offset[0];
	mag[1] -= mag_offset[1];
	mag[2] -= mag_offset[2];
}

/********************************************************************************************/
/* 							PART3: IIC读取数据任务											*/
/********************************************************************************************/

/* 获取加速度数据，获取加速度		*/

void GetAccelTask(void)
{
	uint8_t buf[6], res;
	short raw_acc[3];				// 原始的加速度数据，此处必须用short, 取值范围 -32767~32736，占2字节。int取值范围也相同，但是占4字节
	float fsr;						// 满量程
	const float g = 9.8;

//	while (1)
//	{
		I2C_ReadBytes(MPU6500_ADDR, MPU9250_ACCEL_XOUTH_REG, 6, buf);

		raw_acc[0] = ((uint16_t)buf[0]<<8) | buf[1];  			// x 方向加速度原始值
		raw_acc[1] = ((uint16_t)buf[2]<<8) | buf[3];  			// y 方向加速度原始值
		raw_acc[2] = ((uint16_t)buf[4]<<8) | buf[5];  			// z 方向加速度原始值
		printf("%d %d %d\r\n",raw_acc[0],raw_acc[1],raw_acc[2]);
		if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_2g)						// 选择量程
			fsr = 2.0 * g;
		else if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_4g)
			fsr = 4.0 * g;
		else if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_8g)
			fsr = 8.0 * g;
		else if (MPU9250_ACCEL_FSR == MPU9250_ACCEL_FSR_16g)
			fsr = 16.0 * g;

		acc_[0] = (raw_acc[0]) / 100.0 * (fsr / 327.67);		// x 方向加速度，dof9 于 attitude.c 定义
		acc_[1] = (raw_acc[1]) / 100.0 * (fsr / 327.67);		// y 方向加速度
		acc_[2] = (raw_acc[2]) / 100.0 * (fsr / 327.67);		// z 方向加速度

//		CaliAccelData(acc_);									// 校准

		delay_1ms(20);
//	}
}

/* 获取陀螺仪数据，获取角速度					*/
/* 逆时针旋转角速度为正，顺时针旋转角速度为负	*/

void GetGyroTask(void)
{
	uint8_t buf[6], res;
	float fsr;						// Full Scale Range 满量程
	short raw_gyro[3];				// 原始的陀螺仪数据，此处必须用short, 取值范围 -32767~32736，占2字节。int取值范围也相同，但是占4字节

//	while (1)
//	{
		I2C_ReadBytes(MPU6500_ADDR, MPU9250_GYRO_XOUTH_REG, 6, buf);
		raw_gyro[0] = ((uint16_t)buf[0]<<8) | buf[1];  			// 原始 gyro x
		raw_gyro[1] = ((uint16_t)buf[2]<<8) | buf[3];  			// 原始 gyro y
		raw_gyro[2] = ((uint16_t)buf[4]<<8) | buf[5];			// 原始 gyro z

		if (MPU9250_GYRO_FSR == MPU9250_GYRO_FSR_250)
			fsr = 250.0;
		else if (MPU9250_GYRO_FSR == MPU9250_GYRO_FSR_500)
			fsr = 500.0;
		else if (MPU9250_GYRO_FSR == MPU9250_GYRO_FSR_1000)
			fsr = 1000.0;
		else if (MPU9250_GYRO_FSR == MPU9250_GYRO_FSR_2000)
			fsr = 2000.0;

		gyro_[0] = (float)raw_gyro[0] * (fsr / 3276.70) / -10.0;// 添加负号，使得绕 x 轴逆时针转动为正
		gyro_[1] = (float)raw_gyro[1] * (fsr / 3276.70) / 10.0;	// 乘以 fsr 除以 32767，为了乘积太大引起误差，先除以3276.7再除以10.0。
		gyro_[2] = (float)raw_gyro[2] * (fsr / 3276.70) / 10.0;

//		CaliGyroData(gyro_);									// 校准
		delay_1ms(20);
//	}
}

/* 获取磁力计数据任务							*/
/* mag[0]~mag[2] 分别为x,z,y方向16位磁力计数据	*/
/* !!! 是 x,z,y方向数据，不是 x,y,z 方向，因为HMC5883读取数据顺序就是 x,z,y !!!*/
/* short为2字节，读取的buf[0],buf[1]也是两字节。int为四字节，(int)65534=65534, (short)65534=-2*/
void GetMagTask(void)	// 获取磁力计原始数据
{
	uint8_t buf[6] ={0}, res;
//	while (1)
//	{
		I2C_ReadBytes(AK8963_ADDR, MPU9250_MAG_XOUT_L, 6, buf);	// IIC读取磁力计原始数据

		mag_[0] = ((uint16_t)buf[1]<<8) | buf[0];		// 磁力计 x方向原始数据
		mag_[1] = ((uint16_t)buf[3]<<8) | buf[2];		// 磁力计 y方向原始数据
		mag_[2] = ((uint16_t)buf[5]<<8) | buf[4];		// 磁力计 z方向原始数据
		I2C_WriteByte(AK8963_ADDR, MPU9250_MAG_CNTL1, 0x11);		// 单次测量模式

//		CaliMagData(mag_);

		delay_1ms(20);
//	}
}

/* 得到温度值			*/
/* 返回值:温度值（°C）	*/
/* 此处为使用到			*/
float GetAccelTemperature(void)
{
    uint8_t buf[2];
    short raw_tmp;		// 温度的原始数据
	float tmp;			// 温度值
	I2C_ReadBytes(MPU6500_ADDR, MPU9250_TEMP_OUTH_REG, 2, buf);
    raw_tmp = ((uint16_t)buf[0]<<8) | buf[1];
    tmp = 36.53 + ((double)raw_tmp) / 340.0;
    return tmp;
}

/*********************************************-***********************************************/
/* 							PART4: 外部接口													*/
/********************************************************************************************/

/* 获取磁力计数据接口	*/
void GetMagDataApi(short mag[3])
{
	GetMagTask();
	mag[0] = mag_[0];
	mag[1] = mag_[1];
	mag[2] = mag_[2];
}

/* 获取三轴加速度接口	*/
void GetAccelDataApi(float acc[3])
{
	GetAccelTask();
	acc[0] = acc_[0];
	acc[1] = acc_[1];
	acc[2] = acc_[2];
}

/* 获取三轴角速度接口	*/
void GetGyroDataApi(float gyro[3])
{
	GetGyroTask();
	gyro[0] = gyro_[0];
	gyro[1] = gyro_[1];
	gyro[2] = gyro_[2];
}
