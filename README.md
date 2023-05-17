# Epoch-Remote-Controller

Remote controller for Robocon 2023

主控：STM32F103VET6
无线通信模块：Ebyte E34-2G4D20D nRF24L01芯片2.4G无线串口模块

## 发送控制指令
控制帧结构
|   域  | 通道0  |
| ---- | ----  |
| 偏移  |   0   |
| 长度  |   11   |
| 范围  |   364 -- 1024 -- 1648     |
| 功能  | 遥控器通道0 |

## 监控机器人状态
TODO
