#include "control_task.h"
#include "math.h"

#include "mpu6050.h"
#include "filter.h"
#include "bsp_motor.h"
#include "bsp_key.h" 

static void angle_control(balance_car_t *balance_car);
static void speed_control(balance_car_t *balance_car);
static void turn_control(balance_car_t *balance_car);
static void set_dead_time_pwm(balance_car_t *balance_car);

balance_car_t balance_car;
static TaskHandle_t INS_task_local_handler;
/**
  * @brief   控制任务
  * @param    
  * @retval  balance_car  平衡车控制结构体
 **/
void control_task(void const * argument)
{
	float accyAngle = 0;    //加速度计算出来的角度
	float gyroyReal = 0;    //角速度转换出来的实际值
	
	//获取当前任务的任务句柄
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
	
	//初始化一些变量
	balance_car.control_mode = 0;     //模式为0是并级  1是串级 这个参数没调好
	balance_car.angleAim = 2.5   ;    //目标角度，也就是平衡角度
	balance_car.angleKp  = -200  ;//
	balance_car.angleKd  = -0.4  ;//
	balance_car.speedAim = 0;
	if(0 == balance_car.control_mode)//模式为0是并级
	{
		balance_car.speedKp  = -50;  //-140  -70
		balance_car.speedKi  = balance_car.speedKp/200;//-0.7  -0.35
	}else if(1 == balance_car.control_mode)//模式为1是串级
	{
		balance_car.speedKp  = 0.32;
		balance_car.speedKi  = balance_car.speedKp/200;	
	}
	balance_car.turnAim = 0;
	balance_car.turnKp  = 10;
	balance_car.turnKd  = 0;
	
	mpu6050_init();
	motor_init();
	
	while(1)
	{
		//等待中断中的任务通知
		while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
		//获得6050原始数据
		mpu6050_get_gyro(&balance_car.gyro[0],&balance_car.gyro[1],&balance_car.gyro[2]);
		mpu6050_get_acc(&balance_car.acc[0],&balance_car.acc[1],&balance_car.acc[2]);
		//计算角度
		accyAngle=atan2(balance_car.acc[0],balance_car.acc[2])*180/PI; //加速度计算倾角	
		balance_car.gyroBalance = balance_car.gyro[1];
		gyroyReal=balance_car.gyro[1]/16.4;                            //陀螺仪量程转换	
		Kalman_getAngle(&KalmanY,accyAngle,-gyroyReal,0.01);           //卡尔曼滤波算角度
		balance_car.car_angle = KalmanY.angle;
		
		//遥控器控制相关
		#if RC_CONTROL
		balance_car.rc_data = get_rc_data();
		balance_car.speedAim = (balance_car.rc_data->lx_value - 2048)/64;
		balance_car.turnAim  = (balance_car.rc_data->ry_value - 2048)/64;
		#endif
		
		//计算各个环的PWM
		angle_control(&balance_car);
		speed_control(&balance_car);
		turn_control(&balance_car);
		
		//左右电机赋值
		if(0 == balance_car.control_mode)//模式为0是并级
		{
			balance_car.leftPwm  = balance_car.anglePwm  + balance_car.speedPwm - balance_car.turnPwm;
			balance_car.rightPwm = balance_car.anglePwm  + balance_car.speedPwm + balance_car.turnPwm;
		}else if(1 == balance_car.control_mode)//模式为1是串级
		{
			balance_car.leftPwm  = balance_car.anglePwm  - balance_car.turnPwm;
			balance_car.rightPwm = balance_car.anglePwm  + balance_car.turnPwm;
		}		
		//添加死区控制
		set_dead_time_pwm(&balance_car);
		//角度超过一定角度，就清除PWM
		if(balance_car.car_angle > 40 || balance_car.car_angle < -40)
		{
			balance_car.leftPwm  = 0;
			balance_car.rightPwm = 0;
		}
		//设置实际的值
		motor_set_pwm(balance_car.leftPwm,balance_car.rightPwm);
	}
}

















/**
  * @brief   获得平衡车相关参数
  * @param    
  * @retval  balance_car  平衡车控制结构体
 **/
const balance_car_t *get_balance_car(void)
{
	return &balance_car;
}
/**
  * @brief   设置平衡车速度
  * @param   speed 速度 
  * @retval  
 **/
void set_car_speed(int16_t speed)
{
	balance_car.SpeedAim = speed;
}
/**
  * @brief   设置平衡车转向角度
  * @param   angle 角度
  * @retval  
 **/
void set_car_turn(int16_t angle)
{
	balance_car.TurnAim = angle;
}
/**
  * @brief   设置平衡车PID参数
  * @param   angleKp 角度比例系数 angleKd 角度微分系数 speedKp 速度比例系数
  * @retval  None
 **/
void set_car_pid(float angleKp, float angleKd, float speedKp)
{
	balance_car.angleKp = angleKp;
	balance_car.angleKd = angleKd;
	balance_car.speedKp = speedKp;
	balance_car.speedKi = balance_car.speedKp / 200;
}
/**
  * @brief   死区控制
  * @param   balance_car 平衡车控制结构体
  * @retval  None
 **/
static void set_dead_time_pwm(balance_car_t *balance_car)
{
	if (balance_car == NULL)
	{
		return;
	}
	if (balance_car->leftPWM > 0) {balance_car->leftPWM += 200;}
	else if (balance_car->leftPWM < 0) {balance_car->leftPWM -= 200;}

	if (balance_car->rightPWM > 0) {balance_car->rightPWM += 200;}
	else if (balance_car->rightPWM < 0) {balance_car->rightPWM -= 200;}
}
/**
  * @brief   角度控制 计算出角度环所需的PWM
  * @param   balance_car 平衡车控制结构体
  * @retval  None
 **/
static void angle_control(balance_car_t *balance_car)
{
	float AngleBias;
	int16_t anglePout = 0, angleDout = 0;
	if (balance_car == NULL)
	{
		return;
	}
	if(0 == balance_car->control_mode)
	{
		AngleBias = balance_car->angle_Aim - balance_car->car_angle;
	}else if(1 == balance_car->control_mode)
	{
		AngleBias = balance_car->SpeedPWM + balance_car->angle_Aim - balance_car->car_angle;
	}
	anglePout = AngleBias * balance_car->angleKp;
	angleDout = (AngleBias - balance_car->gyroBalance) * balance_car->angleKd;
	
	balance_car->angle_PWM = anglePout + angleDout;
}
//原有KD公式 kp*bias 如果我要加一个滤波 则公式变为 (1-a)*kp*bias + a*bias_last
/**
  * @brief   速度环控制 计算出速度环所需的PWM
  * @param   balance_car 平衡车控制结构体
  * @retval  None
 **/
static void speed_control(balance_car_t *balance_car)
{
	float a = 0.8;
	float SpeedBias;
	int16_t speedPout = 0, speedIout = 0;
	static int16_t speedBiasLowOut=0,speedBiasLowOutLast=0;
	static int16_t speedAdd = 0;
	if (balance_car == NULL)
	{
		return;
	}
	//读取编码器速度
	balance_car->LeftEncode = -read_encoder(2);
	balance_car->RightEncode = -read_encoder(4);
	balance_car->SpeedNow = (balance_car->LeftEncode + balance_car->RightEncode);
	//滤波
	SpeedBias = balance_car->SpeedAim - balance_car->SpeedNow;
	speedBiasLowOut = (1-a)*SpeedBias + a*speedBiasLowOutLast;
	speedBiasLowOutLast = speedBiasLowOut;
	//积分
	speedAdd += SpeedBias;
	//积分限幅
	if(speedAdd > 10000)
	{
		speedAdd = 10000;
	}
	else if(speedAdd < -10000)
	{
		speedAdd = -10000;
	}
	//如果小车倒下，清除积分
	if(balance_car->car_angle > 40 || balance_car->car_angle < -40)
	{
		speedAdd = 0;
	}
	speedIout = speedAdd * balance_car->speedKi;
	speedPout = speedBiasLowOut * balance_car->speedKp;
	balance_car->SpeedPWM = speedPout + speedIout;
}
/**
  * @brief   转向环控制 计算出转向环所需的PWM
  * @param   balance_car 平衡车控制结构体
  * @retval  None
 **/
static void turn_control(balance_car_t *balance_car)
{
	float TurnBias;
	int16_t turnPout = 0, turnDout = 0;
	if (balance_car == NULL)
	{
		return;
	}
	TurnBias = balance_car->TurnAim - balance_car->TurnNow;
	turnPout = TurnBias * balance_car->turnKp;
	turnDout = (TurnBias - balance_car->gyro[2]) * balance_car->turnKd;
	balance_car->TurnPWM = turnPout + turnDout;
}
/**
  * @brief   中断处理
  * @param    
  * @retval  void
 **/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  
{   
    if(GPIO_Pin == MPU_INT_Pin)  
    {  
		//任务通知唤醒 
		if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }  
}