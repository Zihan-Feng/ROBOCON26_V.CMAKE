/**
 * @file UsbPort.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 封装一个虚拟串口
 * @version 0.1
 * @date 2026-04-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : 单例模式
 * @versioninfo :
 */
#pragma once

#include "lockfree_queue.hpp"

#include <cstddef>
#include <cstdint>

class UsbPort {
public:
  // 数据包结构，最大256字节
  struct Packet {
    uint16_t len{0};
    uint8_t data[256]{};
  };

  // 可观性status监测
  struct Stats {
    uint32_t rx_packets{0};
    uint32_t rx_bytes{0};
    uint32_t rx_drop_packets{0};
    uint32_t tx_packets{0};
    uint32_t tx_bytes{0};
    uint32_t tx_drop_packets{0};
    uint32_t tx_busy_hits{0};
  };

  using RxCallback = void (*)(const uint8_t *data, size_t len, void *user);

  void SetRxCallback(RxCallback cb, void *user) {
    rx_callback_ = cb;
    rx_user_ = user;
  }

  static UsbPort &Instance();

  /**
   * @brief 异步写入usb发送端口
   *
   * @param data
   * @param len
   * @return true
   * @return false
   */
  bool WriteAsync(const uint8_t *data, size_t len);

  /**
   * @brief 读取
   *
   * @param packet
   * @return true
   * @return false
   */
  bool Read(Packet &packet);

  void PumpTx();
  void OnRxFromIsr(const uint8_t *data, size_t len);
  void OnTxCpltFromIsr();

  Stats GetStats() const { return stats_; }

private:
  UsbPort() = default;

  static constexpr size_t kPacketPayload = 256;
  static constexpr size_t kRxQueueDepth = 32;
  static constexpr size_t kTxQueueDepth = 32;

  using PacketQueue = Algorithm::MpscQueue<Packet, 32>;

  // 无锁队列用于接收发送
  PacketQueue rx_queue_;
  PacketQueue tx_queue_;

  Packet tx_staging_{};
  bool tx_staging_valid_{false};
  volatile bool tx_inflight_{false};

  RxCallback rx_callback_ = nullptr;
  void *rx_user_ = nullptr;

  Stats stats_{};
};
