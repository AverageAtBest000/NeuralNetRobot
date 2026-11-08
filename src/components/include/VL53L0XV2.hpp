#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "soc/gpio_num.h"
#include <cstddef>
#include <cstdint>

class VL53L0XV2 {
private:
  static constexpr std::uint8_t ADDRS = 0x29;
  static constexpr std::uint8_t ADDRS_REGISTER = 0x8A;
  static constexpr int I2C_TIMEOUT_MS = 20;
  static constexpr std::uint8_t SYSRANGE_START = 0x00;
  static constexpr std::uint8_t RESULT_INTERRUPT_STATUS = 0x13;
  static constexpr std::uint8_t RESULT_RANGE_STATUS = 0x14;
  static constexpr std::uint8_t SYSTEM_INTERRUPT_CLEAR = 0x0B;

  const gpio_num_t XSHUT_PIN;
  gpio_config_t conf;

  std::uint16_t distance_mm = UINT16_MAX;
  i2c_master_dev_handle_t distance_sensor = nullptr;
  i2c_master_bus_handle_t bus = nullptr;

  bool valid = false;
  std::int64_t sample_time_us = 0;

  void configure_spads();
  void calibrate(std::uint8_t sequence, std::uint8_t start);
  void wait_ready(std::uint8_t addr, std::uint8_t mask);
  void write_register(std::uint8_t addr, std::uint8_t value);
  std::uint8_t read_register(std::uint8_t addr);
  esp_err_t read_register(std::uint8_t addr, std::uint8_t *buffer, std::size_t len);

public:
  // Initializes and calibrates the sensor, then starts continuous ranging.
  VL53L0XV2(i2c_master_bus_handle_t &bus, const gpio_num_t XSHUT_PIN);
  // Returns UINT16_MAX when the cached distance is invalid or older than 100 ms.
  std::uint16_t get_dist_mm();

  bool is_valid(std::uint32_t max_age_ms = 100) const;
  // Returns true only for a new valid measurement. Does not wait for one to finish.
  bool update();

  void driveXSHUT(uint32_t state);

  gpio_config_t getConfig();

  void set_addr(std::uint8_t new_addr);

  void init();
};
