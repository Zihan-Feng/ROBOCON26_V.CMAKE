/**
 * @file XboxRemote.hpp
 * @author 老冯/Keten (2863861004@qq.com)
 * @brief 封装xbox用串口
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

#include "UartPort.hpp"
#include "stm32h7xx_hal.h"

#include "bsp_dwt.h"

#define FRAME_HEAD_0_ESP32 0xFC
#define FRAME_HEAD_1_ESP32 0xFB
#define FRAME_ID_ESP32 0x01 // 示例数据帧ID
#define FRAME_END_0_ESP32 0xFD
#define FRAME_END_1_ESP32 0xFE
#define MAX_DATA_LENGTH_ESP32 28

class XboxRemote {
  enum class rx_state_esp32_t {
    WAITING_FOR_HEADER_0_ESP32,
    WAITING_FOR_HEADER_1_ESP32,
    WAITING_FOR_ID_ESP32,
    WAITING_FOR_LENGTH_ESP32,
    WAITING_FOR_DATA_ESP32,
    WAITING_FOR_CRC_0_ESP32,
    WAITING_FOR_CRC_1_ESP32,
    WAITING_FOR_END_0_ESP32,
    WAITING_FOR_END_1_ESP32
  };

  struct serial_frame_esp32_t {
    uint8_t data_length;
    uint8_t frame_head[2];
    uint8_t frame_id;
    uint16_t crc_calculated;
    union data_buff {
      uint8_t buff_msg[MAX_DATA_LENGTH_ESP32];
    } data_buff;
    union check_code_esp {
      uint16_t crc_code;
      uint8_t crc_buff[2];
    } check_code_e;
    uint8_t frame_end[2];
  };

  struct xbox_controller_data_t {
    // 按键数据（bool类型）
    bool btnY;
    bool btnY_last;
    bool btnB;
    bool btnB_last;
    bool btnA;
    bool btnA_last;
    bool btnX;
    bool btnX_last;
    bool btnShare;
    bool btnShare_last;
    bool btnStart;
    bool btnStart_last;
    bool btnSelect;
    bool btnSelect_last;
    bool btnXbox;
    bool btnXbox_last;
    bool btnLB;
    bool btnLB_last;
    bool btnRB;
    bool btnRB_last;
    bool btnLS;
    bool btnLS_last;
    bool btnRS;
    bool btnRS_last;
    bool btnDirUp;
    bool btnDirUp_last;
    bool btnDirLeft;
    bool btnDirLeft_last;
    bool btnDirRight;
    bool btnDirRight_last;
    bool btnDirDown;
    bool btnDirDown_last;

    // 霍尔值（16位数值）
    uint16_t joyLHori;
    uint16_t joyLVert;
    uint16_t joyRHori;
    uint16_t joyRVert;
    uint16_t trigLT;
    uint16_t trigRT;

    float joyLHori_map;
    float joyLVert_map;
    float joyRHori_map;
    float joyRVert_map;
    float trigLT_map;
    float trigRT_map;
  };

public:
  XboxRemote(UartPort &impl) : impl_(impl) {}

  uint8_t init(void);

  // 处理单个字节，返回帧完整时的Frame ID，否则返回0
  uint8_t processByte(uint8_t byte);

  // 获取最近一次成功解析的控制器数据
  const xbox_controller_data_t &getControllerData() const {
    return controller_data_;
  }

  uint8_t xbox_process(uint8_t byte);

  bool button_continue(bool currentBtnState, bool *lastBtnState);

  bool button_switch(bool currentBtnState, bool *lastBtnState);

private:
  UartPort &impl_;

  // 状态机状态
  rx_state_esp32_t rx_state_{rx_state_esp32_t::WAITING_FOR_HEADER_0_ESP32};

  // 接收缓冲区
  serial_frame_esp32_t rx_frame_{};
  uint8_t rx_temp_data_[MAX_DATA_LENGTH_ESP32 * 4]{};
  uint16_t rx_data_index_{0};

  // 控制器状态
  xbox_controller_data_t controller_data_{};

  // 状态重置
  void resetState();

  // 数据解析回调
  void onFrameComplete(const uint8_t *data, uint8_t len);
};