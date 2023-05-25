/**
  ******************************************************************************
  * @file     remote_control.h
  * @author   Junshuo
  * @version  V1.0
  * @date     2023-05-19
  * @brief    This file contains the headers of remote_control.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __remote_control_h
#define __remote_control_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "user_lib.h"
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	// 遥控器通道值
  int16_t ch[7];  // 摇杆&电位器通道采集
  char sw[2];     // 拨码开关
  char key[8];    // 按键

  // 打包数据
  uint8_t packed[18];

  // 滤波器
  first_order_filter_type_t rocker_lpf[4];  //使用一阶低通滤波消除采样噪声

} rc_info_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void remote_control_init(rc_info_t *rc);
void remote_control_transmit(rc_info_t *rc);

  
#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

