#include "MPU6500.hpp"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <cstdint>

MPU6500::MPU6500(i2c_master_bus_handle_t &bus) {

  // MPU6500 config
  i2c_device_config_t imu_config{};
  imu_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  imu_config.device_address = ADDRS;
  imu_config.scl_speed_hz = 100000;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &imu_config, &imu));

  // wake MPU6500. Send signal to PWR_MGMT_1
  std::uint8_t wake_cmd[] = {0x6B, 0x00};

  ESP_ERROR_CHECK(i2c_master_transmit(imu, wake_cmd, sizeof(wake_cmd), 1000));
}

void MPU6500::update() {
  ESP_ERROR_CHECK(i2c_master_transmit_receive(imu, &regs, sizeof(regs), reads, sizeof(reads), 1000));
  merge_bits();
}

/**
 * @brief Merges high and low bits for a 16 bit measurement
 */
void MPU6500::merge_bits() {
  a_x = static_cast<std::uint16_t>(reads[0] << 8 | reads[1]);
  a_y = static_cast<std::uint16_t>(reads[2] << 8 | reads[3]);
  a_z = static_cast<std::uint16_t>(reads[4] << 8 | reads[5]);

  g_x = static_cast<std::uint16_t>(reads[8] << 8 | reads[9]);
  g_y = static_cast<std::uint16_t>(reads[10] << 8 | reads[11]);
  g_z = static_cast<std::uint16_t>(reads[12] << 8 | reads[13]);
}

/**
 * @brief Acceleration getters
 */
std::uint16_t MPU6500::get_ax() { return a_x; }
std::uint16_t MPU6500::get_ay() { return a_y; }
std::uint16_t MPU6500::get_az() { return a_z; }
/**
 * @brief Gyroscope getters
 * */
std::uint16_t MPU6500::get_gx() { return g_x; }
std::uint16_t MPU6500::get_gy() { return g_y; }
std::uint16_t MPU6500::get_gz() { return g_z; }
