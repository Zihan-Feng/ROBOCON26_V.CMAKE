/**
 * @file control_task.cpp
 * @author 大帅将军
 * @brief 控制任务，遥控/上位机 接口都接入到此，解析后向其他任务发布可能的控制指令
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "control_task.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "bsp_usart.h"
#include "tracking.h"

osThreadId_t ControlTaskHandle;

// 发布底盘控制指令
TypedTopicPublisher<pub_chassis_cmd> chassis_data_pub("chassis_cmd");
pub_chassis_cmd chassis_cmd{};

/* 订阅xbox遥控控制信息 */
static TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox",8);
pub_Xbox_Data control_xbox_cmd{};

// 处理xbox数据，处理为底盘控制指令
void Xbox_Data_Process()
{
  if (ABS(control_xbox_cmd.joyLHori - 32767) > 2000)
  {
    chassis_cmd.linear_x_ = (int)(control_xbox_cmd.joyLHori - 32767) / 32767.0f * MAX_VELOCITY;
  }
  else
  {
    chassis_cmd.linear_x_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyLVert - 32767) > 2000)
  {
    chassis_cmd.linear_y_ = -(int)(control_xbox_cmd.joyLVert - 32767) / 32767.0f * MAX_VELOCITY;
  }
  else
  {
    chassis_cmd.linear_y_ = 0.0f;
  }

  if (ABS(control_xbox_cmd.joyRHori - 32767) > 2000)
  {
    chassis_cmd.omega_ = (int)(control_xbox_cmd.joyRHori - 32767) / 32767.0f * MAX_VELOCITY;
  }
  else
  {
    chassis_cmd.omega_ = 0.0f;
  }
}

void controlInit() { 
  if (!chassis_data_pub.IsValid()) {
    // 发布者初始化失败
    return;
  }
  if (!control_xbox_sub.IsValid()) {
    // 订阅失败
    return;
  }
}

void controlTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();

  controlInit();
  for (;;) {
    
    /* 从xbox数据订阅者中获取数据 */
    if (control_xbox_sub.TryGet(&control_xbox_cmd)) {
      Xbox_Data_Process();
      chassis_data_pub.Publish(chassis_cmd);
    }
    vTaskDelayUntil(&currentTime, 5);
  }
}
