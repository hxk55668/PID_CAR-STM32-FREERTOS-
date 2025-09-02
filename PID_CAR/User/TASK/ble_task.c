/**
  ******************************************************************************
  * @file    ble_task.c
  * @author  LQH Team (snqx-lgh)
  * @version V1.0
  * @date    2025-01-28
  * @brief   蓝牙通信任务模块
  * 
  * 功能概述：
  * - 通过蓝牙接收PID参数并设置平衡小车的控制参数
  * - 实时上传小车姿态角度数据
  * - 支持DMA+空闲中断实现高效数据传输
  * - 包含数据校验机制保证通信可靠性
  * 
  * 协议格式：
  * ------------------------------------------------------------------
  * | 下行指令帧格式（手机->小车，9字节）               |
  * |---------------------------------------------------------------|
  * | 帧头 | 角度KP高8位 | 角度KP低8位 | 角度KD高8位 | 角度KD低8位 | 
  * | 速度KP高8位 | 速度KP低8位 | 校验和 | 帧尾 |
  * |---------------------------------------------------------------|
  * | 0xA5 |     KP_H    |     KP_L    |     KD_H    |     KD_L    |
  * |     Speed_KP_H     |    Speed_KP_L    |  SUM  |   0x5A     |
  * ------------------------------------------------------------------
  * 
  * ------------------------------------------------------------------
  * | 上行数据帧格式（小车->手机，5字节）               |
  * |---------------------------------------------------------------|
  * | 帧头 | 角度值低8位 | 角度值高8位 | 校验和 | 帧尾 |
  * |---------------------------------------------------------------|
  * | 0xA5 | Angle_Low | Angle_High |  SUM  | 0x5A |
  * ------------------------------------------------------------------
  ******************************************************************************
  * @attention
  * 硬件依赖：
  * - USART3 用于蓝牙通信
  * - DMA1 Channel2 用于发送中断处理
  * 
  * <h2><center>&copy; Copyright 2025 LQH, China</center></h2>
  ******************************************************************************
  */

  #include "ble_task.h"
  #include "control_task.h"
  #include "usart.h"
  #include "queue.h"
  
  /* 宏定义 */
  #define BLE_RECEIVE_BUFF_LEN   20  // 蓝牙接收缓冲区长度
  #define BLE_QUEUE_NUM          4   // 消息队列容量
  #define BLE_FRAME_SIZE         9   // 完整数据帧长度
  
  /* 全局变量 */
  uint8_t BLE_Receive_Buff[BLE_RECEIVE_BUFF_LEN] = {0}; // DMA接收缓冲区
  uint8_t BLE_Solve_Buff[BLE_RECEIVE_BUFF_LEN] = {0};   // 数据解析缓冲区
  
  static QueueHandle_t BLE_Message_Queue; // 蓝牙消息队列
  
  /**
	* @brief  蓝牙通信任务主函数
	* @param  argument: FreeRTOS任务参数
	* @retval None
	*/
  void ble_task(void const * argument)
  {
	  /* 本地变量 */
	  uint8_t Receive_Buff[BLE_RECEIVE_BUFF_LEN] = {0}; // 接收数据缓存
	  uint8_t Send_Buff[BLE_RECEIVE_BUFF_LEN] = {0};    // 发送数据缓存
	  int16_t angle = 0;                  // 小车倾斜角度（放大10倍）
	  float angleKpSet = 0, angleKdSet = 0, speedKpSet = 0; // PID参数
	  uint8_t check_sum = 0;               // 校验和
	  const balance_car_t *balance_car_temp = get_car_status(); // 获取小车状态指针
  
	  /* 初始化配置 */
	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);     // 使能串口空闲中断
	  HAL_UART_Receive_DMA(&huart3, BLE_Receive_Buff, BLE_RECEIVE_BUFF_LEN); // 启动DMA接收
	  BLE_Message_Queue = xQueueCreate(BLE_QUEUE_NUM, BLE_RECEIVE_BUFF_LEN); // 创建消息队列
  
	  vTaskDelay(1000); // 等待系统稳定
  
	  /* 任务主循环 */
	  while(1)
	  {
		  if(BLE_Message_Queue != NULL)
		  {
			  // 等待队列数据（阻塞式）
			  if(xQueueReceive(BLE_Message_Queue, Receive_Buff, portMAX_DELAY))
			  {
				  /*---------- 下行数据处理 ----------*/
				  if((Receive_Buff[0] == 0xA5) && (Receive_Buff[8] == 0x5A))
				  {
					  // 计算校验和（字节1-6求和）
					  check_sum = Receive_Buff[1] + Receive_Buff[2] + Receive_Buff[3] 
								+ Receive_Buff[4] + Receive_Buff[5] + Receive_Buff[6];
					  
					  if(check_sum == Receive_Buff[7])
					  {
						  /* 解析PID参数 */
						  angleKpSet = (float)((Receive_Buff[2] << 8) + Receive_Buff[1]);
						  angleKdSet = (float)((Receive_Buff[4] << 8) + Receive_Buff[3]);
						  speedKpSet = (float)((Receive_Buff[6] << 8) + Receive_Buff[5]);
  
						  /* 参数转换 */
						  angleKpSet = -angleKpSet;       // 角度环比例项
						  angleKdSet = -angleKdSet/100;   // 角度环微分项（缩小100倍）
						  speedKpSet = -speedKpSet;       // 速度环比例项
  
						  /* 更新PID参数 */
						  set_car_pid(angleKpSet, angleKdSet, speedKpSet);
					  }
				  }
  
				  /*---------- 上行数据发送 ----------*/
				  angle = (int)(balance_car_temp->car_angle * 10); // 角度值放大10倍
				  Send_Buff[0] = 0xA5;                // 帧头
				  Send_Buff[1] = angle & 0xFF;        // 角度低字节
				  Send_Buff[2] = (angle >> 8) & 0xFF; // 角度高字节
				  Send_Buff[3] = Send_Buff[1] + Send_Buff[2]; // 校验和
				  Send_Buff[4] = 0x5A;                // 帧尾
				  
				  // 通过串口发送数据（阻塞式，超时10ms）
				  HAL_UART_Transmit(&huart3, Send_Buff, 5, 10);
			  }
		  }
	  }
  }
  
  /**
	* @brief  USART3空闲中断处理函数
	* @param  None
	* @retval None
	*/
  void USART3_IRQHandler(void)
  {
	  uint8_t usart3_rx_len = 0;
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
	  if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
	  {
		  __HAL_UART_CLEAR_IDLEFLAG(&huart3);  // 清除空闲中断标志
		  
		  /* 停止DMA接收 */
		  __HAL_DMA_DISABLE(&hdma_usart3_rx);
		  
		  /* 计算接收数据长度 */
		  usart3_rx_len = BLE_RECEIVE_BUFF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		  
		  /* 校验数据长度 */
		  if(usart3_rx_len == BLE_FRAME_SIZE)
		  {
			  // 复制数据到解析缓冲区
			  memcpy(BLE_Solve_Buff, BLE_Receive_Buff, BLE_RECEIVE_BUFF_LEN);
			  
			  // 发送数据到消息队列（从中断上下文）
			  xQueueSendFromISR(BLE_Message_Queue, BLE_Solve_Buff, &xHigherPriorityTaskWoken);
			  
			  // 触发上下文切换（如果需要）
			  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			  
			  // 清空接收缓冲区
			  memset(BLE_Receive_Buff, 0, BLE_RECEIVE_BUFF_LEN);
		  }
		  
		  /* 重新配置DMA */
		  hdma_usart3_rx.Instance->CNDTR = BLE_RECEIVE_BUFF_LEN; // 设置传输数据量
		  __HAL_DMA_ENABLE(&hdma_usart3_rx);                     // 使能DMA
	  }
  }
  
  /**
	* @brief  DMA发送完成中断处理函数
	* @param  None
	* @retval None
	*/
  void DMA1_Channel2_IRQHandler(void)
  {
	  if(__HAL_DMA_GET_FLAG(&hdma_usart3_tx, DMA_FLAG_TC2))
	  {
		  __HAL_DMA_CLEAR_FLAG(&hdma_usart3_tx, DMA_FLAG_TC2); // 清除传输完成标志
		  HAL_UART_DMAStop(&huart3); // 停止DMA传输
	  }
  }