/**
 * @file chassis_task.cpp
 * @author 大帅将军
 * @brief 底盘任务实现
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */

#include "chassis_task.h"
#include "Motor.hpp"
#include "chassis_solution.hpp"
#include "com_config.h"
#include "topic_pool.h"
#include "topics.hpp"

#include <array>

//任务句柄
osThreadId_t ChassisTaskHandle;

//底盘电机实例声明
extern C620Motor chassis_motor1, chassis_motor2, chassis_motor3, chassis_motor4;

//底盘控制命令订阅
static TypedTopicSubscriber<pub_chassis_cmd> chassis_cmd_sub("chassis_cmd", 8);
pub_chassis_cmd chassis_chassis_cmd{};

//底盘解算器实例声明
namespace {
MecanumChassis chassis_solver(chassis_motor1, chassis_motor2, chassis_motor3,
                chassis_motor4);

// 每个轮子的PID参数配置
const std::array<MecanumChassis::SpeedPidParam, MecanumChassis::kWheelCount>
  kWheelPidParams = {
    MecanumChassis::SpeedPidParam(105.0f, 75.0f, 0.20f, 20000.0f, 0.3f,
                    NONE), // 左上
    MecanumChassis::SpeedPidParam(100.0f, 72.0f, 0.15f, 20000.0f, 0.3f,
                    NONE), // 右上
    MecanumChassis::SpeedPidParam(108.0f, 78.0f, 0.22f, 20000.0f, 0.3f,
                    NONE), // 左下
    MecanumChassis::SpeedPidParam(102.0f, 74.0f, 0.18f, 20000.0f, 0.3f,
                    NONE), // 右下
  };
} // namespace

// 用到的初始化
static inline void chassisInit() {
  // 每个电机使用独立 PID 参数
  chassis_solver.configureSpeedPid(kWheelPidParams);

  // 底盘控制命令订阅初始化
  if (!chassis_cmd_sub.IsValid()) {
    // 订阅失败
    return;
  }
}

void chassisTask(void *argument) {
  TickType_t currentTime = xTaskGetTickCount();

  chassisInit();

  for (;;) {
    // 尝试获取最新的底盘控制命令，如果有新命令则进行处理
    if (chassis_cmd_sub.TryGet(&chassis_chassis_cmd)) {
      // Process the received chassis command
    }
    // 进行解算并控制电机
    chassis_solver.run(chassis_chassis_cmd);
    vTaskDelayUntil(&currentTime, 5); // 每1ms执行一次发送任务
  }
}