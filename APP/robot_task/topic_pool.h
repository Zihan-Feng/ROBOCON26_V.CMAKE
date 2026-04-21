/**
 * @file topic_pool.h
 * @author 大帅将军 ，Keten (2863861004@qq.com)
 * @brief
 * 模块所依赖的数据类型结构体，有些模块会依赖这些数据类型结构体进行数据传输，所以移植module层都
 *        必须携带这个包
 * @version 0.1
 * @date 2024-10-03
 *
 * @copyright Copyright (c) 2024
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include "fdcan.h"
#include "usart.h"
#include <stdbool.h>

#pragma pack(1)

typedef struct {
  UART_HandleTypeDef *huart; // 串口句柄
  uint16_t len;              // 数据长度
  void *data_addr; // 数据地址，使用时把地址赋值给这个指针，数值强转为uint8_t
} UART_TxMsg;

typedef struct {
  bool btnY;
  bool btnA;
  bool btnLB;
  bool btnRB;
  uint16_t trigLT;
  uint16_t trigRT;
  bool btnDirUp;
  bool btnDirDown;
  bool btnDirLeft;
  bool btnDirRight;
  bool btnB;
  bool btnX;
  uint16_t joyLHori;
  uint16_t joyLVert;
  uint16_t joyRHori;
  uint16_t joyRVert;
  // 这里填写你需要传输的Xbox按键摇杆等数据
  // bool btnY;
  // bool btnY_last;
  //......

} pub_Xbox_Data;

// 底盘运动指令
typedef struct {
  float linear_x_;
  float linear_y_;
  float omega_;
} pub_chassis_cmd;

#pragma pack()