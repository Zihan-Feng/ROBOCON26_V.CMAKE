/**
 * @file pm20s.hpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

// 占空比配置为0-100
class PM20sServo {
public:
  PM20sServo(TIM_HandleTypeDef &impl, uint8_t channel_id)
      : impl_(impl), channel_id_(channel_id) {}

  void init() { HAL_TIM_PWM_Start(&impl_, channel_id_); }

  void setServoAngle(float angle) {
    uint8_t cmd = 0;
    cmd =  static_cast<uint8_t>(5 + angle/9.f);
    switch (channel_id_) {
    case TIM_CHANNEL_1: {
      impl_.Instance->CCR1 = cmd - 1;
      break;
    }
    case TIM_CHANNEL_2: {
      impl_.Instance->CCR2 = cmd - 1;

      break;
    }
    case TIM_CHANNEL_3: {
      impl_.Instance->CCR3 = cmd - 1;

      break;
    }
    case TIM_CHANNEL_4: {
      impl_.Instance->CCR4 = cmd - 1;

      break;
    }
    case TIM_CHANNEL_5: {
      impl_.Instance->CCR5 = cmd - 1;

      break;
    }
    case TIM_CHANNEL_6: {
      impl_.Instance->CCR6 = cmd - 1;

      break;
    }
    default:
      break;
    }
  }

private:
  // 句柄
  TIM_HandleTypeDef &impl_;
  uint32_t channel_id_;
};