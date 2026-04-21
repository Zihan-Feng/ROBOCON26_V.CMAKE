/**
 * @file ROSCom.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 封装ros上位机协议
 * @version 0.1
 * @date 2026-04-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "ROSCom.hpp"
#include <cstddef>
#include <cstdint>
#include <cstring>

void ROSProtocol::init(void) {
  resetState();
  std::memset(&sensor_bag_, 0, sizeof(sensor_bag_));
  std::memset(&control_bag_, 0, sizeof(control_bag_));
}

uint16_t ROSProtocol::processData(uint8_t byte) {
  // 解析协议
  switch (rx_state_) {
  case ros_protocol_state::CHECK_HEAD1:
    if (byte == HEADER1) {
      rx_state_ = ros_protocol_state::CHECK_HEAD2;
    } else {
      rx_state_ = ros_protocol_state::CHECK_HEAD1;
    }
    break;
  case ros_protocol_state::CHECK_HEAD2:
    if (byte == HEADER2) {
      rx_state_ = ros_protocol_state::CHECK_ID;
    } else {
      rx_state_ = ros_protocol_state::CHECK_HEAD1;
    }
    break;
  case ros_protocol_state::CHECK_ID:
    bag_id_ = (package_id)byte;
    rx_state_ = ros_protocol_state::CHECK_LENGTH;
    break;
  case ros_protocol_state::CHECK_LENGTH:
    rx_tmp_data_length_ = byte;
    rx_state_ = ros_protocol_state::UNPACK_DATA;
    break;
  case ros_protocol_state::UNPACK_DATA:
    // 解包数据
    rx_tmp_data[rx_data_index_++] = byte;
    if (rx_data_index_ >= rx_tmp_data_length_) {
      rx_state_ = ros_protocol_state::CHECK_CRC;
    }
    break;
  case ros_protocol_state::CHECK_CRC:
    // todo:添加crc计算
    rx_state_ = ros_protocol_state::CHECK_TAIL1;
    // break;
  case ros_protocol_state::CHECK_TAIL1:
    if (byte == TAIL1) {
      rx_state_ = ros_protocol_state::CHECK_TAIL2;
    } else {
      rx_state_ = ros_protocol_state::CHECK_HEAD1;
      resetState();
    }
    break;
  case ros_protocol_state::CHECK_TAIL2:
    if (byte == TAIL2) {
      onFrameComplete();
      resetState();
      return static_cast<uint16_t>(bag_id_);
    } else {
      rx_state_ = ros_protocol_state::CHECK_HEAD1;
      resetState();
    }
    break;

  default:
    resetState();
    break;
  }

  return 0;
}

void ROSProtocol::resetState() {
  rx_state_ = ros_protocol_state::CHECK_HEAD1;
  rx_data_index_ = 0;
  rx_tmp_data_length_ = 0;
  std::memset(&rx_tmp_data, 0, sizeof(rx_tmp_data));
}

void ROSProtocol::packSendData(void) {}

void ROSProtocol::onFrameComplete() {
  switch (bag_id_) {
  case package_id::NONE: {
    // NONE
    break;
  }
  case package_id::SENSOR_BAG: {
    if (rx_tmp_data_length_ != sizeof(sensor_bag_))
      break;
    std::memcpy(sensor_bag_.i16_data, rx_tmp_data,
                sizeof(sensor_bag_.i16_data));
    std::memcpy(sensor_bag_.f_data, rx_tmp_data + sizeof(sensor_bag_.i16_data),
                sizeof(sensor_bag_.f_data));
    break;
  }
  case package_id::CONTROL_BAG: {
    if (rx_tmp_data_length_ != sizeof(control_bag_))
      break;
    std::memcpy(control_bag_.i16_data, rx_tmp_data,
                sizeof(control_bag_.i16_data));
    std::memcpy(control_bag_.f_data,
                rx_tmp_data + sizeof(control_bag_.i16_data),
                sizeof(control_bag_.f_data));
    break;
  }
  default:
    break;
  }
}
