/**
 * @file robot.cpp
 * @author 大帅将军
 * @brief 机器人初始化，包含通信外设、消息订阅系统、任务调度
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "robot.h"
#include "Canbus.hpp"
#include "com_config.h"
#include "robot_task.h"
#include "topics.hpp"

/* 更多特色功能可以在这里通过宏开关来决定是否使用 */

void Robot_Init() {
  /* 关闭中断，防止在初始化过程中发生中断 */
  __disable_irq();
  
  // 初始化DWT计时器，提供计时功能
  DWT_Init(SystemCoreClock / 1000000U);

  // 消息订阅系统初始化
  SubPub_Init();

  // 通信外设初始化
  if (comServiceInit() != 0) {
    Error_Handler();
  }


  /* freertos任务调度初始化 */
  osTaskInit();

  /*初始化完成，开启中断 */
  __enable_irq();
}
