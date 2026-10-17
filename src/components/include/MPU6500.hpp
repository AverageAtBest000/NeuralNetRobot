#pragma once

#include "driver/i2c_master.h"
#include <cstdint>

class MPU6500 {
private:
  static constexpr std::uint8_t ADDRS = 0x68;
  i2c_master_dev_handle_t imu = nullptr;
  std::uint8_t reads[14];
  std::uint8_t regs = 0x3B;
  std::uint16_t a_x;
  std::uint16_t a_y;
  std::uint16_t a_z;
  std::uint16_t g_x;
  std::uint16_t g_y;
  std::uint16_t g_z;
  void merge_bits();

public:
  MPU6500(i2c_master_bus_handle_t bus);
  void update();
  std::uint16_t get_ax();
  std::uint16_t get_ay();
  std::uint16_t get_az();
  std::uint16_t get_gx();
  std::uint16_t get_gy();
  std::uint16_t get_gz();
};
