/**
 * @file UartPort.hpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-04-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include "double_buffer.hpp"
#include "lockfree_queue.hpp"
#include "stm32h7xx_hal.h"

#include "memory_map.h"
#include <cstddef>
#include <cstdint>

class UartPort {
public:
  static constexpr size_t kPacketPayloadSize = 64;
  static constexpr size_t kRxQueueDepth = 8;

  struct Packet {
    uint16_t len{0};
    uint8_t data[kPacketPayloadSize]{};
  };

  // 自定义回调
  using RxCallback = void (*)(const uint8_t *data, size_t len, void *user);

  /**
   * @brief Construct a new Uart Port object
   *
   * @param huart
   * @param rx_dma_buf
   * @param rx_dma_buf_size
   * @param tx_dma_buf
   * @param tx_dma_buf_size
   * @param cb
   * @param cb_user
   */
  UartPort(UART_HandleTypeDef *huart, uint8_t *rx_dma_buf,
           size_t rx_dma_buf_size, uint8_t *tx_dma_buf = nullptr,
           size_t tx_dma_buf_size = 0, RxCallback cb = nullptr,
           void *cb_user = nullptr);

  /**
   * @brief
   *
   * @return HAL_StatusTypeDef
   */
  HAL_StatusTypeDef startRxDmaIdle();

  /**
   * @brief
   *
   * @param data
   * @param len
   * @param timeout_ms
   * @return HAL_StatusTypeDef
   */
  HAL_StatusTypeDef write(const uint8_t *data, size_t len,
                          uint32_t timeout_ms = 10);

  /**
   * @brief
   *
   * @param data
   * @param len
   * @return HAL_StatusTypeDef
   */
  HAL_StatusTypeDef writeDma(const uint8_t *data, size_t len);

  bool Read(Packet &packet);

  /**
   * @brief
   *
   */
  void onRxEvent();
  /**
   * @brief
   *
   */
  void onTxCplt();

  void onError(uint32_t error_code);

  UART_HandleTypeDef *handle() const { return huart_; }

  bool txBusy() const { return tx_busy_; }

  static UartPort *fromHandle(UART_HandleTypeDef *huart);

private:
  static constexpr size_t kMaxMap = 4;

  // 保存一个uart实例
  UART_HandleTypeDef *huart_{nullptr};
  uint8_t *rx_dma_buf_{nullptr};
  size_t rx_dma_buf_size_{0};
  size_t last_rx_pos_{0};

  uint8_t tx_fallback_[2] = {0, 0};
  Algorithm::RawData tx_dma_raw_;
  Algorithm::DoubleBuffer tx_dma_buffer_;
  bool tx_use_double_buffer_{false};

  using RxPacketQueue = Algorithm::MpscQueue<Packet, kRxQueueDepth>;
  RxPacketQueue rx_queue_;

  volatile bool tx_busy_{false};

  RxCallback rx_callback_{nullptr};
  void *rx_callback_user_{nullptr};

  static UartPort *map_[kMaxMap];
};