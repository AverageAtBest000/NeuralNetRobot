#pragma once

#include "driver/i2c_master.h"
#include <cstdint>

class MPU6500 {
private:
  static constexpr std::uint8_t ADDRS = 0x68;
  i2c_master_dev_handle_t imu = nullptr;
  static constexpr int I2C_TIMEOUT_MS = 20;
  std::uint8_t reads[14]{};
  std::int16_t a_x = 0;
  std::int16_t a_y = 0;
  std::int16_t a_z = 0;
  std::int16_t g_x = 0;
  std::int16_t g_y = 0;
  std::int16_t g_z = 0;
  float gx_bias = 0.0f;
  float gy_bias = 0.0f;
  float gz_bias = 0.0f;
  bool valid = false;
  std::int64_t sample_time_us = 0;
  void write_register(std::uint8_t addr, std::uint8_t value);
  void merge_bits();

public:
  MPU6500(i2c_master_bus_handle_t &bus);
  // Returns true only when a new sample was read. I2C failures invalidate the sample.
  bool update();
  bool is_valid(std::uint32_t max_age_ms = 100) const;
  // Keep the robot still. Call during startup; blocks for roughly samples * 10 ms.
  bool calibrate_gyro(std::uint16_t samples = 200);
  // Raw sensor counts, preserved for debugging.
  std::int16_t get_ax();
  std::int16_t get_ay();
  std::int16_t get_az();
  std::int16_t get_gx();
  std::int16_t get_gy();
  std::int16_t get_gz();
  // Acceleration in m/s^2 (including gravity), angular velocity in rad/s.
  float get_ax_m_s2() const;
  float get_ay_m_s2() const;
  float get_az_m_s2() const;
  float get_gx_rad_s() const;
  float get_gy_rad_s() const;
  float get_gz_rad_s() const;
};
