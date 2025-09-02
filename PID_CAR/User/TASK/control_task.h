#ifndef _CONTROL_TASK_H
#define _CONTROL_TASK_H

//头文件
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "nrf_task.h"



typedef struct{
	//角度速度环
	int16_t gyro[3];
	int16_t acc[3];

	float car_angle;
	float angle_Aim;
	int16_t angle_PWM;
	int16_t gyroBalance;//平衡角速度
	float angleKp;
	float angleKd;

	//速度环
	int16_t LeftEncode;
	int16_t RightEncode;

	int16_t SpeedAim;
	int16_t SpeedNow;
	int16_t SpeedPWM;
	float speedKp;
	float speedKi;

	//转向环
	int16_t TurnAim;
	int16_t TurnNow;
	int16_t TurnPWM;
	float turnKp;
	float turnKd;

	//pwm输出
	int16_t leftPWM;
	int16_t rightPWM;

	//控制模式
	uint8_t control_mode;
	
	//遥控器数据
	rc_data_t *rc_data;
}balance_car_t;







#endif