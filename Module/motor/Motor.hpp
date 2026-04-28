/**
 * @file Motor.hpp
 * @author Keten (2863861004@qq.com)，大帅将军
 * @brief 电机基类
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :因为达妙电机自带mit控制板，只需要发送目标值，电机板会自己根据当前状态进行闭环控制
 *             所以现在当电机状态切换（模式切换）和目标值变化时，才会调用电机发送can帧，这和大疆电机不同，后者是一直在发送can帧的，
 *             这样可以一定程度上减少can总线的负载，相应的，你只发一针电机也只返回一针，所以无法得知电机当前状态，就连电机任务是否完成都不知道
 *             如果你需要得到电机状态，只需要去cmakelist文件把DM_MOTOR_SINGLE这个宏注释掉就行了
 *             还有一件事，我把达妙电机修改成可以在运动过程中切换模式了，我的发一针可以作为一个保险，例如posWithSpeedControl(1,1)切换到speedControl(1)，是不会发送的因为速度的目标值没有变，
 *             你需要切换到speedControl(0)再切换回speedControl(1)，这样就会发送can帧了，当然如果你不需要切换模式，那就无所谓了，你也可以理解为这是一坨屎一样的代码逻辑，可能你满脑子都在何意味，但是就这样吧
 * @note :
 * @versioninfo :
 */

#pragma once

#include "Canbus.hpp"
#include <cmath>
#include <stdint.h>

#define RAD_2_DEGREE 57.2957795f       // 180/pi  弧度转换成角度
#define DEGREE_2_RAD 0.01745329252f    // pi/180  角度转化成弧度
#define RPM_2_ANGLE_PER_SEC 6.0f       // 360/60,转每分钟 转化 度每秒
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec,转每分钟 转化 弧度每秒

class C610Motor;

class MotorBase {
public:
  MotorBase() = default;

  // 对外接口
  void setMotorCmd(float cmd) {
    if (cmd > max_cmd_)
      cmd = max_cmd_;
    if (cmd < -max_cmd_)
      cmd = -max_cmd_;
    cmd_ = cmd;
  }

  void setMotorReduction(const float config) { reduction_ratio_ = config; }
  void setMaxCmd(const float config) { max_cmd_ = config; }

  float getCurrentSinglePos(void) const { return single_pos_; }
  float getCurrentSumPos(void) const { return sum_pos_; }
  float getCurrentSpeed(void) const { return speed_; }
  float getCurrentTorque(void) const { return torque_; }
  float getCurrentTemperature(void) const { return temperature_; }

  float getRawCurrentSinglePos(void) const { return raw_single_pos_; }
  float getRawCurrentSumPos(void) const { return raw_sum_pos_; }
  float getRawCurrentSpeed(void) const { return raw_speed_; }
  float getRawCurrentTorque(void) const { return raw_torque_; }

  // 电机最原始output指令(速度/位置/电流)
  float cmd_;
  float max_cmd_{99999};

  // 电机减速比
  float reduction_ratio_{1};

  // 电机状态量(最原始)
  float raw_single_pos_{0}; // 单圈位置
  float raw_sum_pos_{0};    // 多圈累加
  float raw_speed_{0};      // 速度
  float raw_torque_{0};     // 力矩

  // 配置减速比(转子->输出端)
  float single_pos_{0};  // 单圈位置
  float sum_pos_{0};     // 多圈累加
  float speed_{0};       // 速度
  float torque_{0};      // 力矩
  float temperature_{0}; // 温度
};

enum DJIMotorCanGroup {
  GROUP1 = 0, // 0x1FF
  GROUP2      // 0x200
};

// 打包大疆电机can帧 - 聚合 4 个电机的命令到单个 8 字节帧
// tx_id=0x200 对应 0x201-0x204; tx_id=0x1FF 对应 0x205-0x208
// 调用方负责提取电机ID和命令值，函数只负责数据聚合
void packDJIMotorCanMsg(uint32_t tx_id, const uint32_t motor_ids[],
                        const int16_t commands[], uint8_t motor_count,
                        uint8_t data[8], uint8_t &len);

class C610Motor : public CanDevice, public MotorBase {
public:
  C610Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id,
            bool tx_is_extid)
      : CanDevice(manager, id, is_extid, tx_id, tx_is_extid) {}

  void init(float reduction_ratio = 36, float max_cmd = 10000.f) {
    setMotorReduction(reduction_ratio);
    setMaxCmd(max_cmd);
  }

  void onRx(const uint8_t data[8], uint8_t len) override {
    if (len < 8)
      return;

    // byte 0-1: 编码器(单圈位置 0-8191)
    encoder_ = (uint16_t)(data[0] << 8 | data[1]);
    if (is_encoder_init) {
      int16_t delta_encoder = encoder_ - last_encoder_;
      // 消圈：大于4096则是反向一圈，小于-4096则是正向一圈
      if (delta_encoder < -4096)
        round_cnt_++;
      else if (delta_encoder > 4096)
        round_cnt_--;
      // 累计编码值
      int32_t total_encoder = round_cnt_ * 8192 + encoder_ - encoder_offset_;

      // 转子侧位置
      raw_single_pos_ = static_cast<float>(encoder_) / encoder_angle_ratio_;
      raw_sum_pos_ = static_cast<float>(total_encoder) / encoder_angle_ratio_;

      // 输出端位置（考虑减速比）
      single_pos_ = raw_single_pos_ / reduction_ratio_;
      sum_pos_ = raw_sum_pos_ / reduction_ratio_;
      single_pos_ = std::fmod(sum_pos_, 360.0f);
      if (single_pos_ < 0.0f)
        single_pos_ += 360.0f;
    } else {
      encoder_offset_ = encoder_;
      is_encoder_init = true;
    }
    last_encoder_ = encoder_;

    // byte 2-3: 转子速度 (int16, 单位: dps/min，取决于驱动器)
    int16_t raw_speed = (int16_t)((data[2] << 8) | data[3]);
    raw_speed_ = static_cast<float>(raw_speed) * RPM_2_RAD_PER_SEC;
    speed_ = raw_speed_ / reduction_ratio_;

    // byte 4-5: 实际电流 (int16, 单位: 0.1A, 范围 -2000~2000 = -200~200A)
    // 电流(A) = value * 0.1
    int16_t raw_current = (int16_t)((data[4] << 8) | data[5]);
    raw_torque_ = static_cast<float>(raw_current) * 0.1f;          // A
    torque_ = raw_torque_ * current_to_torque_ * reduction_ratio_; // 输出端力矩

    // byte 6: 电机温度 (uint8, 单位: °C)
    temperature_ = static_cast<float>(data[6]);
  }

  // buildTx 返回自己的 int16 命令，不发送整个帧
  // 聚合由应用层的 packDJIMotorCanMsg() 负责

  bool buildTx(uint8_t data[8], uint8_t &len) override {
    // C610 不单独发帧，返回 false
    len = 0;
    return false;
  }

  float cmdTrans() { return cmd_ * (10000.f / 10000.0f); }

private:
  // 编码器相关
  bool is_encoder_init{false};
  uint16_t encoder_offset_{0};
  uint16_t encoder_{0};
  uint16_t last_encoder_{0};
  int32_t round_cnt_{0};
  float encoder_angle_ratio_ = 8192.0f / 360.0f;

  // 电流转力矩
  float current_to_torque_{0.0f}; // M3508: 0.2 Nm/A
};

class C620Motor : public CanDevice, public MotorBase {
public:
  C620Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id,
            bool tx_is_extid)
      : CanDevice(manager, id, is_extid, tx_id, tx_is_extid) {}

  void init(float reduction_ratio = 19, float max_cmd = 20000.0f) {
    setMotorReduction(reduction_ratio);
    setMaxCmd(max_cmd);
  }

  void onRx(const uint8_t data[8], uint8_t len) override {
    if (len < 8)
      return;

    // byte 0-1: 编码器(单圈位置 0-8191)
    encoder_ = (uint16_t)(data[0] << 8 | data[1]);
    if (is_encoder_init) {
      int16_t delta_encoder = encoder_ - last_encoder_;
      // 消圈：大于4096则是反向一圈，小于-4096则是正向一圈
      if (delta_encoder < -4096)
        round_cnt_++;
      else if (delta_encoder > 4096)
        round_cnt_--;
      // 累计编码值
      int32_t total_encoder = round_cnt_ * 8192 + encoder_ - encoder_offset_;

      // 转子侧位置
      raw_single_pos_ = static_cast<float>(encoder_) / encoder_angle_ratio_;
      raw_sum_pos_ = static_cast<float>(total_encoder) / encoder_angle_ratio_;

      // 输出端位置（考虑减速比）
      single_pos_ = raw_single_pos_ / reduction_ratio_;
      sum_pos_ = raw_sum_pos_ / reduction_ratio_;
      single_pos_ = std::fmod(sum_pos_, 360.0f);
      if (single_pos_ < 0.0f)
        single_pos_ += 360.0f;
    } else {
      encoder_offset_ = encoder_;
      is_encoder_init = true;
    }
    last_encoder_ = encoder_;

    // byte 2-3: 转子速度 (int16, 单位: rpm，取决于驱动器)
    int16_t raw_speed = (int16_t)((data[2] << 8) | data[3]);
    raw_speed_ = static_cast<float>(raw_speed) * RPM_2_RAD_PER_SEC;
    speed_ = raw_speed_ / reduction_ratio_;

    // byte 4-5: 实际电流
    // 电流(A) = value * 0.1
    int16_t raw_current = (int16_t)((data[4] << 8) | data[5]);
    raw_torque_ = static_cast<float>(raw_current);                 // A
    torque_ = raw_torque_ * current_to_torque_ * reduction_ratio_; // 输出端力矩

    // byte 6: 电机温度 (uint8, 单位: °C)
    temperature_ = static_cast<float>(data[6]);
  }

  float cmdTrans() { return cmd_ * 16384.f / 20000.0f; }

  bool buildTx(uint8_t data[8], uint8_t &len) override {
    // C610 不单独发帧，返回 false
    len = 0;
    return false;
  }

private:
  // 编码器相关
  bool is_encoder_init{false};
  uint16_t encoder_offset_{0};
  uint16_t encoder_{0};
  uint16_t last_encoder_{0};
  int32_t round_cnt_{0};
  float encoder_angle_ratio_ = 8192.0f / 360.0f;

  // 电流转力矩
  float current_to_torque_{0.0f}; // M2006: 0.2 Nm/A
};

class GM6020Motor : public CanDevice, public MotorBase {
public:
  GM6020Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id,
              bool tx_is_extid)
      : CanDevice(manager, id, is_extid, tx_id, tx_is_extid) {}

private:
};

class DM4310Motor : public CanDevice, public MotorBase {
public:
  enum ControlMode : uint8_t {
    Mit = 0x01,
    PosWithSpeed = 0x02,
    Speed = 0x03,
    Psi = 0x04,
  };

  enum MotorModeCmd : uint8_t {
    ModeNone = 0x00,
    Enable = 0x01,
    Disable = 0x02,
    ZeroPosition = 0x03,
    ClearError = 0x04,
    ChangeMode = 0x05,
    SaveConfig = 0x06,
  };

  enum ModeChangeStatus : uint8_t {
    DisableStep = 1,
    ChangeStep = 2,
    SaveStep = 3,
    EnableStep = 4,
    ChangeOkStep = 5,
  };

  DM4310Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id,
              bool tx_is_extid, ControlMode mode = PosWithSpeed)
      : CanDevice(manager, id, is_extid, tx_id, tx_is_extid) {
    tx_base_id_ = tx_id;
    ctrl_mode_ = mode;
    target_mode_ = mode;
  }

  void init(float reduction_ratio = 10.0f, float max_cmd = 18.0f,
            ControlMode mode = PosWithSpeed) {
    setMotorReduction(reduction_ratio);
    setMaxCmd(max_cmd);
    ctrl_mode_ = mode;
    target_mode_ = mode;
    dmMotorEnable();
  }

  void posWithSpeedControl(float pos, float speed) {
    const bool cs_changed = (target_pos_ != pos) || (target_speed_ != speed);
    target_pos_ = pos;
    target_speed_ = speed;
    bool mode_Change = !modeChange(PosWithSpeed, mode_change_state_);
#ifdef DM_MOTOR_SINGLE
    if (!cs_changed || manager_ == nullptr || mode_Change) {
#elif 
    if (manager_ == nullptr || mode_Change) {
#endif
      return;
    }

    CanBus::ClassicPack pack{};
    uint8_t len = 0;
    if (!buildTx(pack.data, len)) {
      return;
    }

    pack.id = tx_id_;
    pack.type = CanBus::Type::STANDARD;
    (void)manager_->addCanMsg(pack);
  }

  void speedControl(float speed) {
    const bool cs_changed = (target_speed_ != speed);
    target_speed_ = speed;
    bool mode_Change = !modeChange(Speed, mode_change_state_);
#ifdef DM_MOTOR_SINGLE
    if (!cs_changed || manager_ == nullptr || mode_Change) {
#elif 
    if (manager_ == nullptr || mode_Change) {
#endif
      return;
    }

    CanBus::ClassicPack pack{};
    uint8_t len = 0;
    if (!buildTx(pack.data, len)) {
      return;
    }

    pack.id = tx_id_;
    pack.type = CanBus::Type::STANDARD;
    (void)manager_->addCanMsg(pack);
  }

  void mitControl(float speed, float pos, float torque, float Kp, float Kd) {
    const bool cs_changed = (target_speed_ != speed) || (target_pos_ != pos) || (target_kp_ != Kp) || (target_kd_ != Kd);
    target_speed_ = speed;
    target_pos_ = pos;
    setMotorCmd(torque);
    target_kp_ = Kp;
    target_kd_ = Kd;
    bool mode_Change = !modeChange(Mit, mode_change_state_);
#ifdef DM_MOTOR_SINGLE
    if (!cs_changed || manager_ == nullptr || mode_Change) {
#elif 
    if (manager_ == nullptr || mode_Change) {
#endif
      return;
    }

    CanBus::ClassicPack pack{};
    uint8_t len = 0;
    if (!buildTx(pack.data, len)) {
      return;
    }

    pack.id = tx_id_;
    pack.type = CanBus::Type::STANDARD;
    (void)manager_->addCanMsg(pack);
  }

  void psiControl(float pos, float speed, float current) {
    const bool cs_changed = (target_pos_ != pos) || (target_speed_ != speed) || (target_current_ != current);
    target_pos_ = pos;
    target_speed_ = speed;
    target_current_ = current;
    bool mode_Change = !modeChange(Psi, mode_change_state_);
#ifdef DM_MOTOR_SINGLE
    if (!cs_changed || manager_ == nullptr || mode_Change) {
#elif 
    if (manager_ == nullptr || mode_Change) {
#endif
      return;
    }

    CanBus::ClassicPack pack{};
    uint8_t len = 0;
    if (!buildTx(pack.data, len)) {
      return;
    }

    pack.id = tx_id_;
    pack.type = CanBus::Type::STANDARD;
    (void)manager_->addCanMsg(pack);
  }

  void dmMotorEnable(void) {
    motor_mode_cmd_ = Enable;

    if (manager_ == nullptr) {
      return;
    }

    CanBus::ClassicPack pack{};
    uint8_t len = 0;
    if (!buildTx(pack.data, len)) {
      return;
    }

    pack.id = tx_id_;
    pack.type = CanBus::Type::STANDARD;
    (void)manager_->addCanMsg(pack);
  }

  void dmMotorDisable(void) {
     motor_mode_cmd_ = Disable; 
    if (manager_ == nullptr) {
      return;
    }

    CanBus::ClassicPack pack{};
    uint8_t len = 0;
    if (!buildTx(pack.data, len)) {
      return;
    }

    pack.id = tx_id_;
    pack.type = CanBus::Type::STANDARD;
    (void)manager_->addCanMsg(pack);
    }

  void dmMotorChangeMode(void){
    motor_mode_cmd_ = ChangeMode;
    if (manager_ == nullptr) {
      return;
    }

    CanBus::ClassicPack pack{};
    uint8_t len = 0;
    if (!buildTx(pack.data, len)) {
      return;
    }

    pack.id = tx_id_;
    pack.type = CanBus::Type::STANDARD;
    (void)manager_->addCanMsg(pack);
  }

  void dmMotorSave(void){
    motor_mode_cmd_ = SaveConfig;
    if (manager_ == nullptr) {
      return;
    }

    CanBus::ClassicPack pack{};
    uint8_t len = 0;
    if (!buildTx(pack.data, len)) {
      return;
    }

    pack.id = tx_id_;
    pack.type = CanBus::Type::STANDARD;
    (void)manager_->addCanMsg(pack);
  }

  void dmMotorZeroPosition(void) { motor_mode_cmd_ = ZeroPosition; }

  void dmMotorClearError(void) { motor_mode_cmd_ = ClearError; }

  uint8_t modeChange(ControlMode mode, ModeChangeStatus status) {
    if (status == DisableStep && ctrl_mode_ != mode) {
      target_mode_ = mode;
      dmMotorDisable();
      mode_change_state_ = ChangeStep;
      return 0;
    }

    if (status == DisableStep && ctrl_mode_ == mode) {
      return 1;
    }

    if (status == ChangeStep) {
      target_mode_ = mode;
      dmMotorChangeMode();
      mode_change_state_ = SaveStep;
      return 0;
    }

    if (status == SaveStep) {
      dmMotorSave();
      mode_change_state_ = EnableStep;
      return 0;
    }

    if (status == EnableStep) {
      ctrl_mode_ = mode;
      dmMotorEnable();
      mode_change_state_ = ChangeOkStep;
      return 0;
    }

    if (status == ChangeOkStep) {
      mode_change_state_ = DisableStep;
      return 1;
    }

    return 0;
  }

  void onRx(const uint8_t data[8], uint8_t len) override {
    if (len < 6)
      return;

    // 达妙 MIT 反馈：byte1-2 位置，byte3-4高4bit 速度，byte4低4bit-5 扭矩
    pos_raw_ = static_cast<uint16_t>((data[1] << 8) | data[2]);
    speed_raw_ = static_cast<uint16_t>((data[3] << 4) | (data[4] >> 4));
    torque_raw_ = static_cast<uint16_t>(((data[4] & 0x0F) << 8) | data[5]);

    raw_single_pos_ = uint_to_float(pos_raw_, MIT_P_MIN, MIT_P_MAX, 16);
    raw_speed_ = uint_to_float(speed_raw_, MIT_V_MIN, MIT_V_MAX, 12);
    raw_torque_ = uint_to_float(torque_raw_, MIT_T_MIN, MIT_T_MAX, 12);

    single_pos_ = raw_single_pos_ / reduction_ratio_;
    raw_sum_pos_ = raw_single_pos_;
    sum_pos_ = single_pos_;
    speed_ = raw_speed_ / reduction_ratio_;
    torque_ = raw_torque_ * reduction_ratio_;
    temperature_ = 0.0f;
  }

  bool buildTx(uint8_t data[8], uint8_t &len) override {
    for (uint8_t i = 0; i < 8; ++i) {
      data[i] = 0;
    }

    // 模式命令帧
    if (motor_mode_cmd_ != ModeNone) {
      len = 8;

      if (motor_mode_cmd_ == ChangeMode || motor_mode_cmd_ == SaveConfig) {
        tx_id_ = 0x7FF;
        data[0] = static_cast<uint8_t>(tx_base_id_ & 0x0F);
        data[1] = static_cast<uint8_t>((tx_base_id_ >> 4) & 0x0F);

        if (motor_mode_cmd_ == ChangeMode) {
          data[2] = 0x55;
          data[3] = 0x0A;
          data[4] = static_cast<uint8_t>(target_mode_);
        } else {
          data[2] = 0xAA;
        }

        motor_mode_cmd_ = ModeNone;
        return true;
      }

      tx_id_ = tx_base_id_ + modeOffsetFromCtrlMode(ctrl_mode_);
      for (uint8_t i = 0; i < 7; ++i) {
        data[i] = 0xFF;
      }

      if (motor_mode_cmd_ == Enable) {
        data[7] = 0xFC;
      } else if (motor_mode_cmd_ == Disable) {
        data[7] = 0xFD;
      } else if (motor_mode_cmd_ == ZeroPosition) {
        data[7] = 0xFE;
      } else {
        data[7] = 0xFB;
      }

      motor_mode_cmd_ = ModeNone;
      return true;
    }

    if (ctrl_mode_ == PosWithSpeed) {
      len = 8;
      tx_id_ = tx_base_id_ + modeOffsetFromCtrlMode(ctrl_mode_);
      const uint8_t *pbuf = reinterpret_cast<const uint8_t *>(&target_pos_);
      const uint8_t *vbuf = reinterpret_cast<const uint8_t *>(&target_speed_);
      data[0] = pbuf[0];
      data[1] = pbuf[1];
      data[2] = pbuf[2];
      data[3] = pbuf[3];
      data[4] = vbuf[0];
      data[5] = vbuf[1];
      data[6] = vbuf[2];
      data[7] = vbuf[3];
      return true;
    }

    if (ctrl_mode_ == Speed) {
      len = 4;
      tx_id_ = tx_base_id_ + modeOffsetFromCtrlMode(ctrl_mode_);
      const uint8_t *vbuf = reinterpret_cast<const uint8_t *>(&target_speed_);
      data[0] = vbuf[0];
      data[1] = vbuf[1];
      data[2] = vbuf[2];
      data[3] = vbuf[3];
      return true;
    }

    if (ctrl_mode_ == Psi) {
      len = 8;
      tx_id_ = tx_base_id_ + modeOffsetFromCtrlMode(ctrl_mode_);

      const uint8_t *pbuf = reinterpret_cast<const uint8_t *>(&target_pos_);
      const uint16_t u16_speed =
          static_cast<uint16_t>(constrain(target_speed_, MIT_V_MIN, MIT_V_MAX) *
                                100.0f);
      const uint16_t u16_current = static_cast<uint16_t>(
          constrain(target_current_, PSI_I_MIN, PSI_I_MAX) * 10000.0f);
      const uint8_t *vbuf = reinterpret_cast<const uint8_t *>(&u16_speed);
      const uint8_t *ibuf = reinterpret_cast<const uint8_t *>(&u16_current);

      data[0] = pbuf[0];
      data[1] = pbuf[1];
      data[2] = pbuf[2];
      data[3] = pbuf[3];
      data[4] = vbuf[0];
      data[5] = vbuf[1];
      data[6] = ibuf[0];
      data[7] = ibuf[1];
      return true;
    }

    // MIT 控制
    len = 8;
    tx_id_ = tx_base_id_ + modeOffsetFromCtrlMode(ctrl_mode_);

    const uint16_t pos_tmp = static_cast<uint16_t>(
        float_to_uint(constrain(target_pos_, MIT_P_MIN, MIT_P_MAX), MIT_P_MIN,
                      MIT_P_MAX, 16));
    const uint16_t vel_tmp = static_cast<uint16_t>(
        float_to_uint(constrain(target_speed_, MIT_V_MIN, MIT_V_MAX), MIT_V_MIN,
                      MIT_V_MAX, 12));
    const uint16_t kp_tmp = static_cast<uint16_t>(float_to_uint(
        constrain(target_kp_, Kp_MIN, Kp_MAX), Kp_MIN, Kp_MAX, 12));
    const uint16_t kd_tmp = static_cast<uint16_t>(float_to_uint(
        constrain(target_kd_, Kd_MIN, Kd_MAX), Kd_MIN, Kd_MAX, 12));
    const uint16_t tor_tmp = static_cast<uint16_t>(
      float_to_uint(constrain(cmd_, MIT_T_MIN, MIT_T_MAX), MIT_T_MIN,
              MIT_T_MAX, 12));

    data[0] = static_cast<uint8_t>(pos_tmp >> 8);
    data[1] = static_cast<uint8_t>(pos_tmp);
    data[2] = static_cast<uint8_t>(vel_tmp >> 4);
    data[3] = static_cast<uint8_t>(((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8));
    data[4] = static_cast<uint8_t>(kp_tmp);
    data[5] = static_cast<uint8_t>(kd_tmp >> 4);
    data[6] = static_cast<uint8_t>(((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8));
    data[7] = static_cast<uint8_t>(tor_tmp);
    return true;
  }

private:
  static constexpr float MIT_P_MIN = -12.5f;
  static constexpr float MIT_P_MAX = 12.5f;
  static constexpr float MIT_V_MIN = -30.0f;
  static constexpr float MIT_V_MAX = 30.0f;
  static constexpr float MIT_T_MIN = -10.0f;
  static constexpr float MIT_T_MAX = 10.0f;
  static constexpr float Kp_MIN = 0.0f;
  static constexpr float Kp_MAX = 500.0f;
  static constexpr float Kd_MIN = 0.0f;
  static constexpr float Kd_MAX = 5.0f;
  static constexpr float PSI_I_MIN = 0.0f;
  static constexpr float PSI_I_MAX = 18.0f;

  static uint16_t modeOffsetFromCtrlMode(ControlMode mode) {
    if (mode == Mit) {
      return 0x000;
    }
    if (mode == PosWithSpeed) {
      return 0x100;
    }
    if (mode == Speed) {
      return 0x200;
    }
    return 0x300;
  }

  static float constrain(float v, float lo, float hi) {
    if (v < lo)
      return lo;
    if (v > hi)
      return hi;
    return v;
  }

  static int float_to_uint(float x1, float x1_min, float x1_max, int bits) {
    const float span = x1_max - x1_min;
    const float offset = x1_min;
    return static_cast<int>((x1 - offset) *
                            ((static_cast<float>((1 << bits) - 1)) / span));
  }

  static float uint_to_float(int x1_int, float x1_min, float x1_max, int bits) {
    const float span = x1_max - x1_min;
    const float offset = x1_min;
    return (static_cast<float>(x1_int) * span /
            static_cast<float>((1 << bits) - 1)) +
           offset;
  }

  uint16_t pos_raw_{0};
  uint16_t speed_raw_{0};
  uint16_t torque_raw_{0};

  float target_pos_{0.0f};
  float target_speed_{0.0f};
  float target_kp_{0.0f};
  float target_kd_{0.0f};
  float target_current_{0.0f};

  ControlMode ctrl_mode_{PosWithSpeed};
  ControlMode target_mode_{PosWithSpeed};
  ModeChangeStatus mode_change_state_{DisableStep};
  MotorModeCmd motor_mode_cmd_{ModeNone};
  uint32_t tx_base_id_{0};
};