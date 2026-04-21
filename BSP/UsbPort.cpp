/**
 * @file UsbPort.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 封装一个虚拟串口
 * @version 0.1
 * @date 2026-04-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : usb
 * cdc虚拟串口初始化，每次cubemx重新生成之后需要重新修改main.c中的初始化顺序，注意修改
 * @versioninfo :
 */
#include "UsbPort.hpp"

#include "UsbPortC.h"
#include "usbd_cdc_if.h"

#include <cstring>

UsbPort &UsbPort::Instance() {
  static UsbPort instance;
  return instance;
}

bool UsbPort::WriteAsync(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0) {
    return false;
  }

  size_t offset = 0;
  while (offset < len) {
    Packet pkt{};
    // 检查是否超过最大发送包长
    const size_t chunk =
        ((len - offset) > kPacketPayload) ? kPacketPayload : (len - offset);

    pkt.len = static_cast<uint16_t>(chunk);
    std::memcpy(pkt.data, data + offset, chunk);

    // 塞入发送队列
    if (tx_queue_.TryPush(pkt) != Algorithm::QueueError::OK) {
      stats_.tx_drop_packets++;
      return false;
    }

    offset += chunk;
  }

  // 主动触发一次发送，避免只入队不下发。
  PumpTx();
  return true;
}

bool UsbPort::Read(Packet &packet) {
  return rx_queue_.TryPop(packet) == Algorithm::QueueError::OK;
}

void UsbPort::PumpTx() {
  if (tx_inflight_) {
    return;
  }

  // 串行进入
  if (!tx_staging_valid_) {
    // 尝试弹出队列元素
    if (tx_queue_.TryPop(tx_staging_) != Algorithm::QueueError::OK) {
      return;
    }
    // 拿到元素，上锁
    tx_staging_valid_ = true;
  }

  const uint8_t result = CDC_Transmit_HS(tx_staging_.data, tx_staging_.len);
  // 发送成功
  if (result == USBD_OK) {
    tx_inflight_ = true;
    tx_staging_valid_ = false;
    stats_.tx_packets++;
    stats_.tx_bytes += tx_staging_.len;
    return;
  }

  if (result == USBD_BUSY) {
    stats_.tx_busy_hits++;
    return;
  }

  stats_.tx_drop_packets++;
  tx_staging_valid_ = false;
}

// usbcdc接收回调中断
void UsbPort::OnRxFromIsr(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0) {
    return;
  }

  size_t offset = 0;
  while (offset < len) {
    Packet pkt{};
    const size_t chunk =
        ((len - offset) > kPacketPayload) ? kPacketPayload : (len - offset);

    pkt.len = static_cast<uint16_t>(chunk);
    std::memcpy(pkt.data, data + offset, chunk);

    if (rx_queue_.TryPush(pkt) != Algorithm::QueueError::OK) {
      stats_.rx_drop_packets++;
      break;
    }

    stats_.rx_packets++;
    stats_.rx_bytes += pkt.len;
    offset += chunk;
  }

  if (rx_callback_) {
    rx_callback_(data, len, rx_user_);
  }
}

// 发送完成中断
void UsbPort::OnTxCpltFromIsr() { tx_inflight_ = false; }

extern "C" void UsbPort_OnRxFromIsr(const uint8_t *data, uint32_t len) {
  UsbPort::Instance().OnRxFromIsr(data, len);
}

extern "C" void UsbPort_OnTxCpltFromIsr(void) {
  UsbPort::Instance().OnTxCpltFromIsr();
}

extern "C" void UsbPort_PumpTx(void) { UsbPort::Instance().PumpTx(); }

extern "C" uint8_t UsbPort_WriteAsync(const uint8_t *data, uint32_t len) {
  return UsbPort::Instance().WriteAsync(data, len) ? 1U : 0U;
}