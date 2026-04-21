/**
 * @file XboxRemote.cpp
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
#include "XboxRemote.hpp"
#include <cstring>

uint8_t XboxRemote::init(void) {
  resetState();
  std::memset(&controller_data_, 0, sizeof(controller_data_));
  return 1;
}

void XboxRemote::resetState() {
  rx_state_ = rx_state_esp32_t::WAITING_FOR_HEADER_0_ESP32;
  rx_data_index_ = 0;
  std::memset(&rx_frame_, 0, sizeof(rx_frame_));
  std::memset(rx_temp_data_, 0, sizeof(rx_temp_data_));
}

uint8_t XboxRemote::processByte(uint8_t byte) {
  // 解析xbox
  switch (rx_state_) {
  case rx_state_esp32_t::WAITING_FOR_HEADER_0_ESP32:
    if (byte == FRAME_HEAD_0_ESP32) {
      rx_frame_.frame_head[0] = byte;
      rx_state_ = rx_state_esp32_t::WAITING_FOR_HEADER_1_ESP32;
    }
    break;

  case rx_state_esp32_t::WAITING_FOR_HEADER_1_ESP32:
    if (byte == FRAME_HEAD_1_ESP32) {
      rx_frame_.frame_head[1] = byte;
      rx_state_ = rx_state_esp32_t::WAITING_FOR_ID_ESP32;
    } else {
      rx_state_ = rx_state_esp32_t::WAITING_FOR_HEADER_0_ESP32;
    }
    break;

  case rx_state_esp32_t::WAITING_FOR_ID_ESP32:
    rx_frame_.frame_id = byte;
    rx_state_ = rx_state_esp32_t::WAITING_FOR_LENGTH_ESP32;
    break;

  case rx_state_esp32_t::WAITING_FOR_LENGTH_ESP32:
    rx_frame_.data_length = byte;
    if (rx_frame_.data_length > MAX_DATA_LENGTH_ESP32) {
      resetState();
      break;
    }
    rx_data_index_ = 0;
    rx_state_ = rx_state_esp32_t::WAITING_FOR_DATA_ESP32;
    break;

  case rx_state_esp32_t::WAITING_FOR_DATA_ESP32:
    rx_temp_data_[rx_data_index_++] = byte;
    if (rx_data_index_ >= rx_frame_.data_length) {
      rx_state_ = rx_state_esp32_t::WAITING_FOR_CRC_0_ESP32;
    }
    break;

  case rx_state_esp32_t::WAITING_FOR_CRC_0_ESP32:
    rx_frame_.check_code_e.crc_buff[0] = byte;
    rx_state_ = rx_state_esp32_t::WAITING_FOR_CRC_1_ESP32;
    break;

  case rx_state_esp32_t::WAITING_FOR_CRC_1_ESP32:
    rx_frame_.check_code_e.crc_buff[1] = byte;
    rx_state_ = rx_state_esp32_t::WAITING_FOR_END_0_ESP32;
    break;

  case rx_state_esp32_t::WAITING_FOR_END_0_ESP32:
    if (byte == FRAME_END_0_ESP32) {
      rx_frame_.frame_end[0] = byte;
      rx_state_ = rx_state_esp32_t::WAITING_FOR_END_1_ESP32;
    } else {
      rx_state_ = rx_state_esp32_t::WAITING_FOR_HEADER_0_ESP32;
    }
    break;

  case rx_state_esp32_t::WAITING_FOR_END_1_ESP32:
    if (byte == FRAME_END_1_ESP32) {
      rx_frame_.frame_end[1] = byte;
      std::memcpy(rx_frame_.data_buff.buff_msg, rx_temp_data_,
                  rx_frame_.data_length);
      onFrameComplete(rx_frame_.data_buff.buff_msg, rx_frame_.data_length);
      resetState();
      return 1;
    }
    rx_state_ = rx_state_esp32_t::WAITING_FOR_HEADER_0_ESP32;
    break;

  default:
    resetState();
    break;
  }

  return 0;
}

void XboxRemote::onFrameComplete(const uint8_t *data, uint8_t len) {
  if (data == nullptr || len < MAX_DATA_LENGTH_ESP32) {
    return;
  }

  // 解析按键数据
  controller_data_.btnY = data[0];
  controller_data_.btnB = data[1];
  controller_data_.btnA = data[2];
  controller_data_.btnX = data[3];
  controller_data_.btnShare = data[4];
  controller_data_.btnStart = data[5];
  controller_data_.btnSelect = data[6];
  controller_data_.btnXbox = data[7];
  controller_data_.btnLB = data[8];
  controller_data_.btnRB = data[9];
  controller_data_.btnLS = data[10];
  controller_data_.btnRS = data[11];
  controller_data_.btnDirUp = data[12];
  controller_data_.btnDirLeft = data[13];
  controller_data_.btnDirRight = data[14];
  controller_data_.btnDirDown = data[15];

  // 解析霍尔传感器值（16位，大端）
  controller_data_.joyLHori = (static_cast<uint16_t>(data[16]) << 8) | data[17];
  controller_data_.joyLVert = (static_cast<uint16_t>(data[18]) << 8) | data[19];
  controller_data_.joyRHori = (static_cast<uint16_t>(data[20]) << 8) | data[21];
  controller_data_.joyRVert = (static_cast<uint16_t>(data[22]) << 8) | data[23];
  controller_data_.trigLT = (static_cast<uint16_t>(data[24]) << 8) | data[25];
  controller_data_.trigRT = (static_cast<uint16_t>(data[26]) << 8) | data[27];
}

uint8_t XboxRemote::xbox_process(uint8_t byte) { return processByte(byte); }

bool XboxRemote::button_continue(bool currentBtnState, bool *lastBtnState) {
  bool result = false;
  if (currentBtnState && !(*lastBtnState)) {
    result = true;
  } else if (!currentBtnState && (*lastBtnState)) {
    result = false;
  }
  *lastBtnState = currentBtnState;
  return result;
}

bool XboxRemote::button_switch(bool currentBtnState, bool *lastBtnState) {
  if (currentBtnState && !(*lastBtnState)) {
    *lastBtnState = currentBtnState;
    return true;
  }
  *lastBtnState = currentBtnState;
  return false;
}
