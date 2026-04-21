/**
 * @file Canbus.hpp
 * @author Keten (2863861004@qq.com)
 * @brief fdcan 总线抽象
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

#include "fdcan.h"

#include "lockfree_queue.hpp"
#include "stm32h7xx_hal_fdcan.h"
#include <atomic>
#include <cstdint>

void canFilterInit(FDCAN_HandleTypeDef *hcan, uint32_t idType, uint32_t fifo,
                   uint32_t id, uint32_t maskId);

void bspCanInit(FDCAN_HandleTypeDef *hcan);

class CanBus;

/**
 * @brief Can设备基类
 *
 */
class CanDevice {
public:
  CanDevice(CanBus *manager, uint32_t id = 0, bool is_extid = false,
            uint32_t tx_id = 0, bool tx_is_extid = false)
      : is_extid_(is_extid), id_(id), tx_id_(tx_id), tx_is_extid_(tx_is_extid),
        manager_(manager) {}

  virtual ~CanDevice() = default;

  uint32_t id() const { return id_; }

  bool isExtId() const { return is_extid_; }

  void setCanId(uint32_t id, bool is_extid) {
    id_ = id;
    is_extid_ = is_extid;
  }

  virtual bool matchRx(const FDCAN_RxHeaderTypeDef &rx_header) const {
    const bool frame_is_ext = (rx_header.IdType == FDCAN_EXTENDED_ID);
    if (frame_is_ext != is_extid_) {
      return false;
    }
    return rx_header.Identifier == id_;
  }

  virtual void onRx(const uint8_t data[8], uint8_t len) {
    (void)data;
    (void)len;
  }

  virtual bool buildTx(uint8_t data[8], uint8_t &len) {
    (void)data;
    len = 0;
    return false;
  }

  bool is_extid_;
  uint32_t id_;

  // 发送id
  uint32_t tx_id_;
  bool tx_is_extid_;

  // 保存一个Manager引用
  CanBus *manager_{nullptr};
};

/**
 * @brief Can 总线Manager类
 *
 */
class CanBus {
public:
  enum class Type : uint8_t { STANDARD = 0, EXTENDED = 1 };

  typedef struct __attribute__((packed)) {
    uint32_t id;
    Type type;
    uint8_t data[8];
  } ClassicPack;

  CanBus(FDCAN_HandleTypeDef &impl) : impl_(impl) {}

  static CanBus *instanceByIndex(uint8_t idx);

  uint8_t init(void);

  uint8_t registerDevice(CanDevice *device);

  uint8_t addCanMsg(const ClassicPack &pack);

  // 处理接收中断
  void processRxInterrupt(uint32_t fifo);

  // 处理发送中断
  void processTxInterrupt();

  uint32_t hardwareTxQueueEmptySize() {
    return HAL_FDCAN_GetTxFifoFreeLevel(&impl_);
  }

  // 保证操作串行化
  void txService();

private:
  // hal实例
  FDCAN_HandleTypeDef &impl_;
  // 维护发送队列
  Algorithm::MpscQueue<ClassicPack, 8> tx_queue_;

  std::atomic<uint32_t> tx_lock_{0};
  std::atomic<uint32_t> tx_pend_{0};

  // 总共有3个fdcan
  static CanBus *map[3];

  // can总线最多挂6个设备
  CanDevice *device_[6];

  uint32_t txMailbox;
};
