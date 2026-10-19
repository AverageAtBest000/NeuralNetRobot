#pragma once

#include "driver/i2c_master.h"
#include <cstdint>

class MPU6500 {
private:
  static constexpr std::uint8_t ADDRS = 0x68;
  i2c_master_dev_handle_t imu = nullptr;
  std::uint8_t reads[14];
  std::uint8_t regs = 0x3B;
  std::int16_t a_x;
  std::int16_t a_y;
  std::int16_t a_z;
  std::int16_t g_x;
  std::int16_t g_y;
  std::int16_t g_z;
  void merge_bits();

public:
  MPU6500(i2c_master_bus_handle_t &bus);
  void update();
  std::int16_t get_ax();
  std::int16_t get_ay();
  std::int16_t get_az();
  std::int16_t get_gx();
  std::int16_t get_gy();
  std::int16_t get_gz();
};
