#include "MPU6500.hpp"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cstdint>

MPU6500::MPU6500(i2c_master_bus_handle_t &bus) {

  // MPU6500 config
  i2c_device_config_t imu_config{};
  imu_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  imu_config.device_address = ADDRS;
  imu_config.scl_speed_hz = 100000;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &imu_config, &imu));

  write_register(0x6B, 0x80); // Reset before applying configuration.

  vTaskDelay(pdMS_TO_TICKS(100));
  write_register(0x6B, 0x01); // Wake nusing the gyro clock
  write_register(0x6C, 0x00); // Enable all axes
  write_register(0x19, 0x09); // 100 Hz sample rate with the filters enabled
  write_register(0x1A, 0x03); // Gyro low-pass filter
  write_register(0x1B, 0x00); // +/-250 degrees/s, 131 counts per degree/s
  write_register(0x1C, 0x00); // +/-2 g, 16384 counts per g
  write_register(0x1D, 0x03); // Accelerometer low-pass filter
  write_register(0x38, 0x01); // Enable data-ready status
  vTaskDelay(pdMS_TO_TICKS(100));
}

void MPU6500::write_register(std::uint8_t addr, std::uint8_t value) {
  const std::uint8_t cmd[]{addr, value};
  ESP_ERROR_CHECK(i2c_master_transmit(imu, cmd, sizeof(cmd), I2C_TIMEOUT_MS));
}

bool MPU6500::update() {
  const std::uint8_t reg = 0x3A;
  std::uint8_t sample[15]{}; // Data-ready status followed by all six axes and temperature.
  if (i2c_master_transmit_receive(imu, &reg, 1, sample, sizeof(sample), I2C_TIMEOUT_MS) != ESP_OK) {
    valid = false;
    return false;
  }
  if ((sample[0] & 0x01) == 0)
    return false;

  std::copy(sample + 1, sample + 15, reads);
  merge_bits();
  sample_time_us = esp_timer_get_time();
  valid = true;
  return true;
}

bool MPU6500::is_valid(std::uint32_t max_age_ms) const {
  return valid && esp_timer_get_time() - sample_time_us <= static_cast<std::int64_t>(max_age_ms) * 1000;
}

bool MPU6500::calibrate_gyro(std::uint16_t samples) {
  if (samples == 0)
    return false;

  std::int64_t sum_x = 0, sum_y = 0, sum_z = 0;
  for (std::uint16_t i = 0; i < samples; ++i) {
    const std::int64_t start = esp_timer_get_time();
    while (!update()) {
      if (esp_timer_get_time() - start >= 100000)
        return false;
      vTaskDelay(1);
    }
    sum_x += g_x;
    sum_y += g_y;
    sum_z += g_z;
    vTaskDelay(std::max(pdMS_TO_TICKS(10), TickType_t{1}));
  }

  gx_bias = static_cast<float>(sum_x) / samples;
  gy_bias = static_cast<float>(sum_y) / samples;
  gz_bias = static_cast<float>(sum_z) / samples;
  return true;
}

/**
 * @brief Merges high and low bits for a 16 bit measurement
 */
void MPU6500::merge_bits() {
  a_x = static_cast<std::int16_t>(reads[0] << 8 | reads[1]);
  a_y = static_cast<std::int16_t>(reads[2] << 8 | reads[3]);
  a_z = static_cast<std::int16_t>(reads[4] << 8 | reads[5]);

  g_x = static_cast<std::int16_t>(reads[8] << 8 | reads[9]);
  g_y = static_cast<std::int16_t>(reads[10] << 8 | reads[11]);
  g_z = static_cast<std::int16_t>(reads[12] << 8 | reads[13]);
}

/**
 * @brief Acceleration getters
 */
std::int16_t MPU6500::get_ax() { return a_x; }
std::int16_t MPU6500::get_ay() { return a_y; }
std::int16_t MPU6500::get_az() { return a_z; }
/**
 * @brief Gyroscope getters
 * */
std::int16_t MPU6500::get_gx() { return g_x; }
std::int16_t MPU6500::get_gy() { return g_y; }
std::int16_t MPU6500::get_gz() { return g_z; }

float MPU6500::get_ax_m_s2() const { return a_x * (9.80665f / 16384.0f); }
float MPU6500::get_ay_m_s2() const { return a_y * (9.80665f / 16384.0f); }
float MPU6500::get_az_m_s2() const { return a_z * (9.80665f / 16384.0f); }
float MPU6500::get_gx_rad_s() const { return (g_x - gx_bias) * (0.01745329252f / 131.0f); }
float MPU6500::get_gy_rad_s() const { return (g_y - gy_bias) * (0.01745329252f / 131.0f); }
float MPU6500::get_gz_rad_s() const { return (g_z - gz_bias) * (0.01745329252f / 131.0f); }
