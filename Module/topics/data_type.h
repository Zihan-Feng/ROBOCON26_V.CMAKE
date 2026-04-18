/**
 * @file data_type.h
 * @author Keten (2863861004@qq.com)
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
#include <stdbool.h>

typedef struct {
  bool btnY;
  bool btnA;
  bool btnLB;
  bool btnRB;
  uint16_t trigLT;
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

typedef struct
{
    float linear_x;// x方向速度 m/s
    float linear_y;// y方向速度 m/s
    float Omega;   // 转动速度 rad/s
    uint8_t Status;// 底盘状态
    uint8_t Move;// 底盘运动方式
    uint8_t ctrl;// 底盘控制模式
    uint8_t if_rev_ros;// 是否接收ros上位机数据
}pub_Control_Data;

#pragma pack()