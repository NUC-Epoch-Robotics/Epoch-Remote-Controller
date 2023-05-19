/**
  ******************************************************************************
  * @file     remote_control.c
  * @author   Junshuo
  * @version  V1.0
  * @date     2023-05-19
  * @brief    遥控器发送
  ******************************************************************************
  * @attention
  * 
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "remote_control.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "user_lib.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
#define ADC_CHN 4 //nummber of ADC channels

#define CHN0_MIN 0
#define CHN0_MAX 4095 
#define CHN0_MIN 0
#define CHN1_MAX 4095 
#define CHN0_MIN 0
#define CHN2_MAX 4095
#define CHN0_MIN 0 
#define CHN3_MAX 4095 
/* Private  variables ---------------------------------------------------------*/
uint32_t stickDatBuf[ADC_CHN]; //摇杆通道采集
rc_info_t rc_info;

/* Extern   variables ---------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_tx;
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/**
  * @brief  遥控器初始化
  * @param None
  * @retval None
  */
void remote_control_init(void)
{
  HAL_ADC_Start_DMA(&hadc1, stickDatBuf, ADC_CHN);
  HAL_TIM_Base_Start(&htim3);

}
       
/**
  * @brief  遥控器18字节控制帧打包
  * @param ch 遥控器通道原始数值
  * @param buff 遥控器控制帧
  * @retval None
  */
void remote_control_packup(uint16_t* ch, uint8_t* buff)
{
  if (ch == NULL || buff == NULL)
  {
    return;
  }

  buff[0] = ch[0] & 0xFF;                                     // 通道0低8位
  buff[1] = ((ch[0] >> 8) & 0x07) | ((ch[1] << 3) & 0xF8);    // 通道0高3位 + 通道1低5位
  buff[2] = ((ch[1] >> 5) & 0x3F) | ((ch[2] << 6) & 0xC0);    // 通道1高6位 + 通道2低2位
  buff[3] = (ch[2] >> 2) & 0xFF;                              // 通道2中间8位
  buff[4] = ((ch[2] >> 10) & 0x01) | ((ch[3] << 1) & 0xFE);   // 通道2高1位 + 通道3低7位
  buff[5] = ((ch[3] >> 7) & 0x0F) | (0x00 & 0xF0);            // 通道3高4位 + 4位填充
  buff[6] = 0x00; // 填充字节
  buff[7] = 0x00; // 填充字节
  buff[8] = 0x00; // 填充字节
  buff[9] = 0x00; // 填充字节
  buff[10] = 0x00; // 填充字节
  buff[11] = 0x00; // 填充字节
  buff[12] = 0x00; // 填充字节
  buff[13] = 0x00; // 填充字节
  buff[14] = 0x00; // 填充字节
  buff[15] = 0x00; // 填充字节
  buff[16] = 0x00; // 填充字节
  buff[17] = 0x00; // 填充字节

}


/**
  * @brief  遥控器数据发送
  * @param rc structure to save handled rc data
  * @retval None
  */
void remote_control_transmit(rc_info_t *rc)
{
  
  remote_control_packup(rc->ch, rc->packed);

  //等待DMA发送完成
  // while (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY)
  // {
  // }
  HAL_UART_Transmit(&huart1, rc->packed, 18, 20); //Timeout给太低会导致发送不全

}


/**
* @brief  Conversion complete callback in non blocking mode
* @param  hadc: ADC handle
* @retval None
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    // 将ADC采集到的数据处理成摇杆通道数值
    for (int i = 0; i < ADC_CHN; i++)
    {
      rc_info.ch[i] = stickDatBuf[i] / 2;
    }

    for (int i = 0; i < ADC_CHN; i++)
    {
      if (rc_info.ch[i] > 1684)
      {
        rc_info.ch[i] = 1684;
      }
      if (rc_info.ch[i] < 364)
      {
        rc_info.ch[i] = 364;
      }
    }
    rc_info.ch[0] = 1024;
    rc_info.ch[1] = 1024;


  }
}

/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/
