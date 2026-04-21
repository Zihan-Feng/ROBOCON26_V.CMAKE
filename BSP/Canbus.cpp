/**
 * @file Canbus.cpp
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
#include "Canbus.hpp"
#include "main.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_fdcan.h"
#include <cstdint>

namespace {

uint8_t dlcToBytes(uint32_t dlc) {
  switch (dlc) {
  case FDCAN_DLC_BYTES_0:
    return 0;
  case FDCAN_DLC_BYTES_1:
    return 1;
  case FDCAN_DLC_BYTES_2:
    return 2;
  case FDCAN_DLC_BYTES_3:
    return 3;
  case FDCAN_DLC_BYTES_4:
    return 4;
  case FDCAN_DLC_BYTES_5:
    return 5;
  case FDCAN_DLC_BYTES_6:
    return 6;
  case FDCAN_DLC_BYTES_7:
    return 7;
  case FDCAN_DLC_BYTES_8:
  default:
    return 8;
  }
}

int8_t busIndexFromHandle(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan == &hfdcan1) {
    return 0;
  }
  if (hfdcan == &hfdcan2) {
    return 1;
  }
  if (hfdcan == &hfdcan3) {
    return 2;
  }
  return -1;
}

} // namespace

CanBus *CanBus::map[3] = {nullptr, nullptr, nullptr};

/**
 * @brief 配置过滤器
 *
 * @param hcan
 * @param idType 接收扩展帧or标准帧
 * @param bank
 * @param fifo
 * @param id
 * @param maskId
 */
void canFilterInit(FDCAN_HandleTypeDef *hcan, uint32_t idType, uint32_t fifo,
                   uint32_t id, uint32_t maskId) {
  FDCAN_FilterTypeDef sFilterConfig = {0};

  sFilterConfig.FilterConfig = fifo; // 消息过滤器配置为接收FIFO0或FIFO1
  sFilterConfig.FilterID1 = id;      // 过滤器ID1
  sFilterConfig.FilterID2 = maskId;  // 过滤器ID2
  sFilterConfig.FilterIndex = 0;     // 过滤器编号
  sFilterConfig.FilterType = FDCAN_FILTER_MASK; // 掩码模式
  sFilterConfig.IdType = idType;                // 标准帧或拓展帧
  sFilterConfig.IsCalibrationMsg = 0;           // 不是标定消息
  sFilterConfig.RxBufferIndex = 0;

  if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  // 配置全局过滤器
  HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

// can外设初始化
void bspCanInit(FDCAN_HandleTypeDef *hcan) {
  // 使能can外设中断
  if (HAL_FDCAN_ActivateNotification(
          hcan,
          FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
              FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE,
          0) != HAL_OK) {
    Error_Handler();
  }

  // 使能can发送完成中断
  if (HAL_FDCAN_TT_ActivateNotification(hcan, FDCAN_IT_TX_FIFO_EMPTY) !=
      HAL_OK) {
    Error_Handler();
  }

  // 使能can外设
  if (HAL_FDCAN_Start(hcan) != HAL_OK) {
    Error_Handler();
  }
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) {
    return;
  }
  // 拿到canbus
  const int8_t idx = busIndexFromHandle(hfdcan);
  CanBus *bus =
      (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
  // 分发对应的can帧
  if (bus != nullptr) {
    bus->processRxInterrupt(FDCAN_RX_FIFO0);
  }
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo1ITs) {
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0) {
    return;
  }

  const int8_t idx = busIndexFromHandle(hfdcan);
  CanBus *bus =
      (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
  if (bus != nullptr) {
    bus->processRxInterrupt(FDCAN_RX_FIFO1);
  }
}

extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan,
                                                   uint32_t BufferIndexes) {
  const int8_t idx = busIndexFromHandle(hfdcan);
  CanBus *bus =
      (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
  if (bus != nullptr) {
    bus->txService();
  }
}

extern "C" void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
  const int8_t idx = busIndexFromHandle(hfdcan);
  CanBus *bus =
      (idx >= 0) ? CanBus::instanceByIndex(static_cast<uint8_t>(idx)) : nullptr;
  if (bus != nullptr) {
    bus->txService();
  }
}

extern "C" void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
  // 清除错误标志位
  // hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
  while(1){}
}

uint32_t error_state = 0;

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                              uint32_t ErrorStatusITs) {
                                                error_state = ErrorStatusITs;
}

CanBus *CanBus::instanceByIndex(uint8_t idx) {
  if (idx >= 3U) {
    return nullptr;
  }
  return map[idx];
}

uint8_t CanBus::init(void) {
  const int8_t idx = busIndexFromHandle(&impl_);
  if (idx < 0) {
    return 1;
  }

  map[idx] = this;
  for (uint8_t i = 0; i < 6; ++i) {
    device_[i] = nullptr;
  }
  txMailbox = 0;

  return 0;
}

uint8_t CanBus::registerDevice(CanDevice *device) {
  if (device == nullptr) {
    return 1;
  }

  for (uint8_t i = 0; i < 6; ++i) {
    if (device_[i] == device) {
      return 0;
    }
  }

  for (uint8_t i = 0; i < 6; ++i) {
    if (device_[i] == nullptr) {
      device_[i] = device;
      return 0;
    }
  }

  return 2;
}

uint8_t CanBus::addCanMsg(const ClassicPack &pack) {
  if (tx_queue_.TryPush(pack) != Algorithm::QueueError::OK) {
    return 1;
  }

  txService();
  return 0;
}

// 处理接收中断
void CanBus::processRxInterrupt(uint32_t fifo) {
  FDCAN_RxHeaderTypeDef rx_header = {0};
  uint8_t data[8] = {0};

  if (HAL_FDCAN_GetRxMessage(&impl_, fifo, &rx_header, data) != HAL_OK) {
    return;
  }

  // 拿到can数据段长度
  const uint8_t len = dlcToBytes(rx_header.DataLength);
  for (uint8_t i = 0; i < 6; ++i) {
    if (device_[i] == nullptr) {
      continue;
    }
    // 设备轮询
    if (!device_[i]->matchRx(rx_header)) {
      continue;
    }
    // 查到设备则调用其回调
    device_[i]->onRx(data, len);
  }
}

// 处理发送中断
void CanBus::processTxInterrupt() { txService(); }

// can发送硬件的串行化
void CanBus::txService() {

  // 进来尝试拿到锁
  if (tx_lock_.exchange(1, std::memory_order_acquire) != 0) {
    // 拿不到则设置忙碌
    tx_pend_.store(1, std::memory_order_release);
    return;
  }

  do {
    // 清除一次busy pend
    tx_pend_.store(0, std::memory_order_release);

    // 然后硬件发送
    // 如果有可用fifo
    while (HAL_FDCAN_GetTxFifoFreeLevel(&impl_) > 0U) {
      ClassicPack pack = {0};
      if (tx_queue_.TryPop(pack) != Algorithm::QueueError::OK) {
        break;
      }

      FDCAN_TxHeaderTypeDef tx_header = {0};
      tx_header.Identifier = pack.id;
      tx_header.IdType =
          (pack.type == Type::EXTENDED) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
      tx_header.TxFrameType = FDCAN_DATA_FRAME;
      tx_header.DataLength = FDCAN_DLC_BYTES_8;
      tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
      tx_header.BitRateSwitch = FDCAN_BRS_OFF;
      tx_header.FDFormat = FDCAN_CLASSIC_CAN;
      tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
      tx_header.MessageMarker = 0;

      // 调用发送,如果失败,标志busy pend,自动进行重试
      if (HAL_FDCAN_AddMessageToTxFifoQ(&impl_, &tx_header, pack.data) !=
          HAL_OK) {
        tx_pend_.store(1, std::memory_order_release);
        break;
      }
    }
  } while (tx_pend_.exchange(0, std::memory_order_acq_rel) != 0);

  // 处理完释放锁
  tx_lock_.store(0, std::memory_order_release);
}
