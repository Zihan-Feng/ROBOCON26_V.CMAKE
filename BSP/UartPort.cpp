/**
 * @file UartPort.cpp
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

#include "UartPort.hpp"

#include <cstring>

UartPort *UartPort::map_[UartPort::kMaxMap] = {nullptr, nullptr, nullptr,
                                               nullptr};

UartPort::UartPort(UART_HandleTypeDef *huart, uint8_t *rx_dma_buf,
                   size_t rx_dma_buf_size, uint8_t *tx_dma_buf,
                   size_t tx_dma_buf_size, RxCallback cb, void *cb_user)
    : huart_(huart), rx_dma_buf_(rx_dma_buf), rx_dma_buf_size_(rx_dma_buf_size),
      tx_dma_raw_{(tx_dma_buf != nullptr && tx_dma_buf_size >= 2)
                      ? static_cast<void *>(tx_dma_buf)
                      : static_cast<void *>(tx_fallback_),
                  (tx_dma_buf != nullptr && tx_dma_buf_size >= 2)
                      ? tx_dma_buf_size
                      : sizeof(tx_fallback_)},
      tx_dma_buffer_(tx_dma_raw_), rx_callback_(cb),
      rx_callback_user_(cb_user) {
  tx_use_double_buffer_ =
      (tx_dma_buf != nullptr && tx_dma_buf_size >= 2 &&
       (tx_dma_buf_size % 2U == 0U) && tx_dma_buffer_.Size() > 0U);

  for (size_t i = 0; i < kMaxMap; ++i) {
    if (map_[i] == nullptr) {
      map_[i] = this;
      break;
    }
  }
}

HAL_StatusTypeDef UartPort::startRxDmaIdle() {
  if (huart_ == nullptr || rx_dma_buf_ == nullptr || rx_dma_buf_size_ == 0) {
    return HAL_ERROR;
  }
  last_rx_pos_ = 0;
  return HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_dma_buf_, rx_dma_buf_size_);
}

HAL_StatusTypeDef UartPort::write(const uint8_t *data, size_t len,
                                  uint32_t timeout_ms) {
  if (huart_ == nullptr || data == nullptr || len == 0) {
    return HAL_ERROR;
  }
  if (len > 0xFFFFU) {
    return HAL_ERROR;
  }
  return HAL_UART_Transmit(huart_, const_cast<uint8_t *>(data),
                           static_cast<uint16_t>(len), timeout_ms);
}

HAL_StatusTypeDef UartPort::writeDma(const uint8_t *data, size_t len) {
  if (huart_ == nullptr || data == nullptr || len == 0) {
    return HAL_ERROR;
  }
  if (len > 0xFFFFU) {
    return HAL_ERROR;
  }

  if (!tx_use_double_buffer_) {
    if (tx_busy_) {
      return HAL_BUSY;
    }
    tx_busy_ = true;
    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(
        huart_, const_cast<uint8_t *>(data), static_cast<uint16_t>(len));
    if (ret != HAL_OK) {
      tx_busy_ = false;
    }
    return ret;
  }

  if (!tx_busy_) {
    if (!tx_dma_buffer_.FillActive(data, len)) {
      return HAL_ERROR;
    }

    tx_busy_ = true;
    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(
        huart_, tx_dma_buffer_.ActiveBuffer(),
        static_cast<uint16_t>(tx_dma_buffer_.GetActiveLength()));
    if (ret != HAL_OK) {
      tx_busy_ = false;
    }
    return ret;
  }

  if (tx_dma_buffer_.HasPending()) {
    return HAL_BUSY;
  }

  if (!tx_dma_buffer_.FillPending(data, len)) {
    return HAL_BUSY;
  }

  return HAL_OK;
}

bool UartPort::Read(Packet &packet) {
  return rx_queue_.TryPop(packet) == Algorithm::QueueError::OK;
}

void UartPort::onRxEvent() {
  if (huart_ == nullptr || huart_->hdmarx == nullptr) {
    return;
  }

  size_t curr_pos = rx_dma_buf_size_ - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
  if (curr_pos == last_rx_pos_) {
    return;
  }

  auto pushRxSpan = [this](const uint8_t *data, size_t len) {
    if (data == nullptr || len == 0) {
      return;
    }

    size_t offset = 0;
    while (offset < len) {
      Packet packet{};
      size_t part = len - offset;
      if (part > sizeof(packet.data)) {
        part = sizeof(packet.data);
      }

      packet.len = static_cast<uint16_t>(part);
      std::memcpy(packet.data, data + offset, part);

      if (rx_queue_.TryPush(packet) != Algorithm::QueueError::OK) {
        break;
      }

      offset += part;
    }
  };

  if (curr_pos > last_rx_pos_) {
    pushRxSpan(rx_dma_buf_ + last_rx_pos_, curr_pos - last_rx_pos_);
    if (rx_callback_ != nullptr) {
      rx_callback_(rx_dma_buf_ + last_rx_pos_, curr_pos - last_rx_pos_,
                   rx_callback_user_);
    }
  } else {
    pushRxSpan(rx_dma_buf_ + last_rx_pos_, rx_dma_buf_size_ - last_rx_pos_);
    if (rx_callback_ != nullptr) {
      rx_callback_(rx_dma_buf_ + last_rx_pos_, rx_dma_buf_size_ - last_rx_pos_,
                   rx_callback_user_);
    }
    if (curr_pos > 0) {
      pushRxSpan(rx_dma_buf_, curr_pos);
      if (rx_callback_ != nullptr) {
        rx_callback_(rx_dma_buf_, curr_pos, rx_callback_user_);
      }
    }
  }

  last_rx_pos_ = curr_pos;
}

void UartPort::onTxCplt() {
  if (!tx_use_double_buffer_) {
    tx_busy_ = false;
    return;
  }

  if (!tx_dma_buffer_.HasPending()) {
    tx_busy_ = false;
    return;
  }

  tx_dma_buffer_.Switch();

  HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(
      huart_, tx_dma_buffer_.ActiveBuffer(),
      static_cast<uint16_t>(tx_dma_buffer_.GetActiveLength()));
  if (ret != HAL_OK) {
    tx_busy_ = false;
  }
}

void UartPort::onError(uint32_t error_code) {
  if (huart_ == nullptr || error_code == HAL_UART_ERROR_NONE) {
    return;
  }

  // TX 出错后释放 busy，避免上层持续卡在 HAL_BUSY。
  tx_busy_ = false;

  // 先停掉当前 DMA，再清错误标志并重启 RX DMA。
  (void)HAL_UART_DMAStop(huart_);
  __HAL_UART_CLEAR_PEFLAG(huart_);
  __HAL_UART_CLEAR_FEFLAG(huart_);
  __HAL_UART_CLEAR_NEFLAG(huart_);
  __HAL_UART_CLEAR_OREFLAG(huart_);
  __HAL_UART_CLEAR_IDLEFLAG(huart_);
  huart_->ErrorCode = HAL_UART_ERROR_NONE;

  last_rx_pos_ = 0;
  (void)startRxDmaIdle();
}

UartPort *UartPort::fromHandle(UART_HandleTypeDef *huart) {
  if (huart == nullptr) {
    return nullptr;
  }
  for (size_t i = 0; i < kMaxMap; ++i) {
    if (map_[i] != nullptr && map_[i]->handle() == huart) {
      return map_[i];
    }
  }
  return nullptr;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t) {
  UartPort *port = UartPort::fromHandle(huart);
  if (port != nullptr) {
    port->onRxEvent();
  }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  UartPort *port = UartPort::fromHandle(huart);
  if (port != nullptr) {
    port->onTxCplt();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  UartPort *port = UartPort::fromHandle(huart);
  if (port != nullptr) {
    port->onError(huart->ErrorCode);
  }
}
