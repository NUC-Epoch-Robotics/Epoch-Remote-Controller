/**
  ******************************************************************************
  * @file    remote_control.h
  * @author  Junshuo
  * @version V1.0
  * @date   
  * @brief   This file contains the headers of remote_control.c
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
/* Exported types ------------------------------------------------------------*/
typedef struct
{
  /* rocker channel information */
	int16_t ch[5];
  /* left and right lever information */
  char sw[2];
  uint8_t packed[18];
} rc_info_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void remote_control_init(void);
void remote_control_packup(uint16_t* ch, uint8_t* buff);
void remote_control_transmit(rc_info_t *rc);

  
#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

