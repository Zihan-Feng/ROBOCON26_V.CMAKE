/**
 * @file ROSCom.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 封装ros上位机协议
 * @version 0.1
 * @date 2026-04-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :  注意 ros 和 32
 * 的交互，如果ros上位机程序不能做到数据源同步发送，那么推荐用包id来指明
 * 这里用2个包id 来指示2种下发的包类型来示范
 * 对于数据长度一般双方规定好（当然这不意味着不可以发送可变包长的数据，只是解析会比较难写）
 * @versioninfo :
 */
#pragma once

#include "UartPort.hpp"
#include "UsbPort.hpp"
#include <cstddef>
#include <cstdint>

#define MAX_ROS_DATA_LENGTH                                                    \
  128 // 其实没啥参考意义，只是标注数据段缓冲区大小最大为128字节

#define HEADER1 0xFF
#define HEADER2 0xFE
#define TAIL1 0xFD
#define TAIL2 0xFB

class ROSProtocol {
  enum class ros_protocol_state {
    CHECK_HEAD1 = 0,
    CHECK_HEAD2,
    CHECK_ID,
    CHECK_LENGTH,
    UNPACK_DATA,
    CHECK_CRC,
    CHECK_TAIL1,
    CHECK_TAIL2
  };

  enum class package_id {
    NONE = 0,
    SENSOR_BAG = 1,
    CONTROL_BAG = 2,
  };

#pragma pack(1)

  /* 接收 */
  struct SensorBag {
    union {
      uint8_t byte[2];
      int16_t data;
    } i16_data[2];

    union {
      uint8_t byte[4];
      float data;
    } f_data[4];
  };

  struct ControlBag {
    union {
      uint8_t byte[2];
      int16_t data;
    } i16_data[4];

    union {
      uint8_t byte[4];
      float data;
    } f_data[2];
  };

  /* 发送 */
#pragma pack()

public:
  // 两种含参构造
  ROSProtocol(UartPort *uart_impl, UsbPort *usb_impl)
      : uart_impl_(uart_impl), usb_impl_(usb_impl) {}

  void init(void);

  // 解析回调
  uint16_t processData(uint8_t byte);

  // 获取最近一次成功解析的数据
  const SensorBag &getSensorBagData() const { return sensor_bag_; }
  const ControlBag &getControlBagData() const { return control_bag_; }

  void packSendData(void);

private:
  void resetState();

  void onFrameComplete();

private:
  // 实例
  UartPort *uart_impl_{nullptr};
  UsbPort *usb_impl_{nullptr};

  // 解包状态
  ros_protocol_state rx_state_{};
  uint16_t rx_data_index_{0};
  uint8_t rx_tmp_data[MAX_ROS_DATA_LENGTH];
  uint16_t rx_tmp_data_length_{};

  // id
  package_id bag_id_{};

  // bag data
  SensorBag sensor_bag_{};
  ControlBag control_bag_{};
};