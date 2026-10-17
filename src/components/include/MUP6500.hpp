#pragma once

#include "driver/i2c_master.h"
#include <cstdint>

class MPU6500 {
private:
  static constexpr std::uint8_t ADDRS = 0x68;
  i2c_master_dev_handle_t imu = nullptr;
  std::uint8_t reads[6];
  std::uint8_t reg = 0x3B;
  std::uint16_t a_x;
  std::uint16_t a_y;
  std::uint16_t a_z;

public:
  MPU6500(i2c_master_bus_handle_t bus);
  void read();
  void merge_bits();
  std::uint8_t get_ax();
  std::uint8_t get_ay();
  std::uint8_t get_az();
};
