/**
  ******************************************************************************
  * @file     remote_control.c
  * @author   Junshuo
  * @version  V1.0
  * @date     2023-05-25
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
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
#define ADC_CHN 7 // nummber of ADC channels
#define ONE_ADC_PERIOD 0.05f // 多长时间ADC进行一次转换

#define CHN_RANGE 660
#define CHN_MID 1024
#define CHN_MIN (CHN_MID - CHN_RANGE)
#define CHN_MAX (CHN_MID + CHN_RANGE)
// 右侧-航模摇杆-X 
#define CHN1_ADC_IN 5
#define CHN1_MIN CHN_MIN
#define CHN1_MAX CHN_MAX
#define CHN1_UP 4095
#define CHN1_MID 2048
#define CHN1_DOWN 0
// 右侧-航模摇杆-Y
#define CHN2_ADC_IN 6
#define CHN2_MIN CHN_MIN
#define CHN2_MAX CHN_MAX
#define CHN2_UP 4095
#define CHN2_MID 2048
#define CHN2_DOWN 0
// 左侧-四维摇杆-X
#define CHN3_ADC_IN 3
#define CHN3_MIN CHN_MIN
#define CHN3_MAX CHN_MAX
#define CHN3_UP 4095
#define CHN3_MID 2048
#define CHN3_DOWN 0
// 左侧-四维摇杆-Y
#define CHN4_ADC_IN 4
#define CHN4_MIN CHN_MIN
#define CHN4_MAX CHN_MAX
#define CHN4_UP 4095
#define CHN4_MID 2048
#define CHN4_DOWN 0
// 左侧-四维摇杆-Z
#define CHN0_ADC_IN 1
#define CHN0_MIN CHN_MIN
#define CHN0_MAX CHN_MAX 
#define CHN0_UP 4095
#define CHN0_MID 2048
#define CHN0_DOWN 0
// 左侧-电位器
#define CHN5_ADC_IN 2
// 右侧-电位器
#define CHN6_ADC_IN 7

//摇杆滤波系数
#define ROCKER_LPF 0.01f
/* Private  variables ---------------------------------------------------------*/
uint32_t stickDatBuf[ADC_CHN]; //摇杆通道采集
rc_info_t rc_info;

static const uint8_t CHN_ADC_IN[7] = {CHN0_ADC_IN, CHN1_ADC_IN, CHN2_ADC_IN, CHN3_ADC_IN, CHN4_ADC_IN, CHN5_ADC_IN, CHN6_ADC_IN};
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
void remote_control_init(rc_info_t *rc)
{

  // 滤波器初始化
  const static fp32 rocker_lpf[1] = {ROCKER_LPF};
  for (int i = 0; i < ADC_CHN; i++)
  {
    first_order_filter_init(&rc->rocker_lpf[i], ONE_ADC_PERIOD, rocker_lpf);
  }

  HAL_ADC_Start_DMA(&hadc1, stickDatBuf, ADC_CHN);
  HAL_TIM_Base_Start(&htim3);

}


/**
  * @brief  摇杆映射
  * @param in 摇杆原始数据
  * @param up 摇杆上限
  * @param mid 摇杆中间值
  * @param down 摇杆下限
  * @retval None
  */
int16_t rocker_correct(int16_t in, int16_t up, int16_t mid, int16_t down)
{
  
  if (in >= mid)
  {
    in = (in - mid) * CHN_RANGE / (up - mid) + CHN_MID;
  }
  else if (in < mid)
  {
    in = (in - mid) * CHN_RANGE / (mid - down) + CHN_MID;
  }

  return in;

}


/**
  * @brief  通道原始数据处理，包括限幅、滤波等
  * @param ch 遥控器通道
  * @retval None
  */
void channel_process(rc_info_t *rc)
{

  // 摇杆数值映射
  rc_info.ch[0] = rocker_correct(rc_info.ch[0], CHN0_UP, CHN0_MID, CHN0_DOWN);
  rc_info.ch[1] = rocker_correct(rc_info.ch[1], CHN1_UP, CHN1_MID, CHN1_DOWN);
  rc_info.ch[2] = rocker_correct(rc_info.ch[2], CHN2_UP, CHN2_MID, CHN2_DOWN);
  rc_info.ch[3] = rocker_correct(rc_info.ch[3], CHN3_UP, CHN3_MID, CHN3_DOWN);
  
  // 通道滤波
  for (int i = 0; i < ADC_CHN; i++)
  {
    first_order_filter_cali(&rc->rocker_lpf[i], rc_info.ch[i]);
  }

  // 限幅
  rc_info.ch[0] = int16_constrain(rc_info.ch[0], CHN0_MIN, CHN0_MAX);
  rc_info.ch[1] = int16_constrain(rc_info.ch[1], CHN1_MIN, CHN1_MAX);
  rc_info.ch[2] = int16_constrain(rc_info.ch[2], CHN2_MIN, CHN2_MAX);
  rc_info.ch[3] = int16_constrain(rc_info.ch[3], CHN3_MIN, CHN3_MAX);

}


/**
  * @brief  遥控器18字节控制帧打包
  * @param ch 遥控器通道原始数值
  * @param buff 遥控器控制帧
  * @retval None
  */
void remote_control_packup(uint16_t* ch, char* key, char* sw, uint8_t* buff)
{
  if (ch == NULL || buff == NULL)
  {
    return;
  }

  buff[0] = ch[1] & 0xFF;                                     // 通道1低8位
  buff[1] = ((ch[1] >> 8) & 0x07) | ((ch[2] << 3) & 0xF8);    // 通道1高3位 + 通道2低5位
  buff[2] = ((ch[2] >> 5) & 0x3F) | ((ch[3] << 6) & 0xC0);    // 通道2高6位 + 通道3低2位
  buff[3] = (ch[3] >> 2) & 0xFF;                              // 通道3中间8位
  buff[4] = ((ch[3] >> 10) & 0x01) | ((ch[4] << 1) & 0xFE);   // 通道3高1位 + 通道4低7位               
  buff[5] = ((ch[4] >> 7) & 0x0F) | 
            ((sw[0] << 4) & 0x30) | ((sw[1] << 6) & 0xC0);    // 通道4高4位 + SW-1 + SW-2
  buff[6] = ch[5] & 0xFF;                                     // 通道5低8位
  buff[7] = ((ch[5] >> 8) & 0x07);                            // 通道5高3位
  buff[8] = ch[6] & 0xFF;                                     // 通道6低8位
  buff[9] = ((ch[6] >> 8) & 0x07);                            // 通道6高3位
  buff[10] = 0x00;                                            // 填充字节
  buff[11] = 0x00;                                            // 填充字节
  buff[12] = 0x00;                                            // 填充字节
  buff[13] = 0x00;                                            // 填充字节
  buff[14] = (key[0]<<7) | (key[1]<<6) | (key[2]<<5) | (key[3]<<4) |
             (key[4]<<3) | (key[5]<<2) | (key[6]<<1) | key[7];// 8个按键
  buff[15] = 0x00;                                            // 填充字节
  buff[16] = ch[0] & 0xFF;                                    // 通道0低8位
  buff[17] = ((ch[0] >> 8) & 0x07);                           // 通道0高3位

}


/**
  * @brief  遥控器数据发送
  * @param rc structure to save handled rc data
  * @retval None
  */
void remote_control_transmit(rc_info_t *rc)
{
  
  remote_control_packup(rc->ch, rc->key, rc->sw, rc->packed);
  HAL_UART_Transmit(&huart1, rc->packed, 18, 20); //使用阻塞式发送，保证每帧都发送成功
  //后续可以改成DMA发送，但是我没弄成功

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
      rc_info.ch[i] = stickDatBuf[CHN_ADC_IN[i] - 1];
    }
    channel_process(&rc_info);


  }
}


/**
  * @brief  This function handles EXTI interrupt request.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_1_Pin)
  {
    rc_info.key[0] = HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin);
  }
  else if (GPIO_Pin == KEY_2_Pin)
  {
    rc_info.key[1] = HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin);
  }
  else if (GPIO_Pin == KEY_3_Pin)
  {
    rc_info.key[2] = HAL_GPIO_ReadPin(KEY_3_GPIO_Port, KEY_3_Pin);
  }
  else if (GPIO_Pin == KEY_4_Pin)
  {
    rc_info.key[3] = HAL_GPIO_ReadPin(KEY_4_GPIO_Port, KEY_4_Pin);
  }
  else if (GPIO_Pin == KEY_5_Pin)
  {
    rc_info.key[4] = HAL_GPIO_ReadPin(KEY_5_GPIO_Port, KEY_5_Pin);
  }
  else if (GPIO_Pin == KEY_6_Pin)
  {
    rc_info.key[5] = HAL_GPIO_ReadPin(KEY_6_GPIO_Port, KEY_6_Pin);
  }
  else if (GPIO_Pin == KEY_7_Pin)
  {
    rc_info.key[6] = HAL_GPIO_ReadPin(KEY_7_GPIO_Port, KEY_7_Pin);
  }
  else if (GPIO_Pin == KEY_8_Pin)
  {
    rc_info.key[7] = HAL_GPIO_ReadPin(KEY_8_GPIO_Port, KEY_8_Pin);
  }
  else if((GPIO_Pin == SW_1_1_Pin) | (GPIO_Pin == SW_1_2_Pin))
  {
    if ((HAL_GPIO_ReadPin(SW_1_1_GPIO_Port, SW_1_1_Pin) == GPIO_PIN_SET) && 
        (HAL_GPIO_ReadPin(SW_1_2_GPIO_Port, SW_1_2_Pin) == GPIO_PIN_RESET))
    {
      rc_info.sw[0] = 3;
    }
    else if ((HAL_GPIO_ReadPin(SW_1_1_GPIO_Port, SW_1_1_Pin) == GPIO_PIN_RESET) && 
             (HAL_GPIO_ReadPin(SW_1_2_GPIO_Port, SW_1_2_Pin) == GPIO_PIN_SET))
    {
      rc_info.sw[0] = 2;
    }
    else
    {
      rc_info.sw[0] = 1;
    }
  }
  else if ((GPIO_Pin == SW_2_1_Pin) | (GPIO_Pin == SW_2_2_Pin))
  {
    if ((HAL_GPIO_ReadPin(SW_2_1_GPIO_Port, SW_2_1_Pin) == GPIO_PIN_SET) && 
        (HAL_GPIO_ReadPin(SW_2_2_GPIO_Port, SW_2_2_Pin) == GPIO_PIN_RESET))
    {
      rc_info.sw[1] = 3;
    }
    else if ((HAL_GPIO_ReadPin(SW_2_1_GPIO_Port, SW_2_1_Pin) == GPIO_PIN_RESET) && 
             (HAL_GPIO_ReadPin(SW_2_2_GPIO_Port, SW_2_2_Pin) == GPIO_PIN_SET))
    {
      rc_info.sw[1] = 2;
    }
    else
    {
      rc_info.sw[1] = 1;
    }
  }
  else
  {
    // do nothing
  }
 
}


/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/
