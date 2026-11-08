#include "VL53L0XV2.hpp"
#include "GA12N20.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_num.h"
#include <cstdint>

VL53L0XV2::VL53L0XV2(i2c_master_bus_handle_t &bus, const gpio_num_t XSHUT_PIN) : XSHUT_PIN(XSHUT_PIN), bus(bus) {
  i2c_device_config_t dist_config{};
  dist_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dist_config.device_address = ADDRS;
  dist_config.scl_speed_hz = 100000;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dist_config, &distance_sensor));

  conf = {
      .pin_bit_mask = 1ULL << XSHUT_PIN,
      .mode = GPIO_MODE_OUTPUT,
  };
}

void VL53L0XV2::init() {
  ESP_ERROR_CHECK(read_register(0xC0) == 0xEE ? ESP_OK : ESP_ERR_INVALID_RESPONSE);
  write_register(0x89, read_register(0x89) | 0x01); // 2.8 V I/O mode.
  write_register(0x88, 0x00);

  write_register(0x80, 0x01);
  write_register(0xFF, 0x01);
  write_register(0x00, 0x00);
  const std::uint8_t stop_variable = read_register(0x91);
  write_register(0x00, 0x01);
  write_register(0xFF, 0x00);
  write_register(0x80, 0x00);

  write_register(0x60, read_register(0x60) | 0x12);
  write_register(0x44, 0x00); // Signal rate limit: 0.25 MCPS in 9.7 format.
  write_register(0x45, 0x20);
  write_register(0x01, 0xFF);
  configure_spads();

  // ST's default tuning settings. Register order and page changes matter.
  const std::uint8_t tuning[][2]{
      {0xFF, 0x01},
      {0x00, 0x00},
      {0xFF, 0x00},
      {0x09, 0x00},
      {0x10, 0x00},
      {0x11, 0x00},
      {0x24, 0x01},
      {0x25, 0xFF},
      {0x75, 0x00},
      {0xFF, 0x01},
      {0x4E, 0x2C},
      {0x48, 0x00},
      {0x30, 0x20},
      {0xFF, 0x00},
      {0x30, 0x09},
      {0x54, 0x00},
      {0x31, 0x04},
      {0x32, 0x03},
      {0x40, 0x83},
      {0x46, 0x25},
      {0x60, 0x00},
      {0x27, 0x00},
      {0x50, 0x06},
      {0x51, 0x00},
      {0x52, 0x96},
      {0x56, 0x08},
      {0x57, 0x30},
      {0x61, 0x00},
      {0x62, 0x00},
      {0x64, 0x00},
      {0x65, 0x00},
      {0x66, 0xA0},
      {0xFF, 0x01},
      {0x22, 0x32},
      {0x47, 0x14},
      {0x49, 0xFF},
      {0x4A, 0x00},
      {0xFF, 0x00},
      {0x7A, 0x0A},
      {0x7B, 0x00},
      {0x78, 0x21},
      {0xFF, 0x01},
      {0x23, 0x34},
      {0x42, 0x00},
      {0x44, 0xFF},
      {0x45, 0x26},
      {0x46, 0x05},
      {0x40, 0x40},
      {0x0E, 0x06},
      {0x20, 0x1A},
      {0x43, 0x40},
      {0xFF, 0x00},
      {0x34, 0x03},
      {0x35, 0x44},
      {0xFF, 0x01},
      {0x31, 0x04},
      {0x4B, 0x09},
      {0x4C, 0x05},
      {0x4D, 0x04},
      {0xFF, 0x00},
      {0x44, 0x00},
      {0x45, 0x20},
      {0x47, 0x08},
      {0x48, 0x28},
      {0x67, 0x00},
      {0x70, 0x04},
      {0x71, 0x01},
      {0x72, 0xFE},
      {0x76, 0x00},
      {0x77, 0x00},
      {0xFF, 0x01},
      {0x0D, 0x01},
      {0xFF, 0x00},
      {0x80, 0x01},
      {0x01, 0xF8},
      {0xFF, 0x01},
      {0x8E, 0x01},
      {0x00, 0x01},
      {0xFF, 0x00},
      {0x80, 0x00},
  };
  for (const auto &setting : tuning)
    write_register(setting[0], setting[1]);

  write_register(0x0A, 0x04); // Interrupt status reports a new measurement.
  write_register(0x84, read_register(0x84) & ~0x10);
  write_register(SYSTEM_INTERRUPT_CLEAR, 0x01);

  calibrate(0x01, 0x41);      // VHV calibration.
  calibrate(0x02, 0x01);      // Phase calibration.
  write_register(0x01, 0xF8); // Keep the default sequence and its timeout settings.

  write_register(0x80, 0x01);
  write_register(0xFF, 0x01);
  write_register(0x00, 0x00);
  write_register(0x91, stop_variable);
  write_register(0x00, 0x01);
  write_register(0xFF, 0x00);
  write_register(0x80, 0x00);
  write_register(SYSRANGE_START, 0x02); // Continuous back-to-back ranging.
}

void VL53L0XV2::configure_spads() {
  write_register(0x80, 0x01);
  write_register(0xFF, 0x01);
  write_register(0x00, 0x00);
  write_register(0xFF, 0x06);
  write_register(0x83, read_register(0x83) | 0x04);
  write_register(0xFF, 0x07);
  write_register(0x81, 0x01);
  write_register(0x80, 0x01);
  write_register(0x94, 0x6B);
  write_register(0x83, 0x00);
  wait_ready(0x83, 0xFF);
  write_register(0x83, 0x01);
  const std::uint8_t spad_info = read_register(0x92);

  write_register(0x81, 0x00);
  write_register(0xFF, 0x06);
  write_register(0x83, read_register(0x83) & ~0x04);
  write_register(0xFF, 0x01);
  write_register(0x00, 0x01);
  write_register(0xFF, 0x00);
  write_register(0x80, 0x00);

  std::uint8_t spad_map[6]{};
  ESP_ERROR_CHECK(read_register(0xB0, spad_map, sizeof(spad_map)));
  write_register(0xFF, 0x01);
  write_register(0x4F, 0x00);
  write_register(0x4E, 0x2C);
  write_register(0xFF, 0x00);
  write_register(0xB6, 0xB4);

  const int first_spad = (spad_info & 0x80) ? 12 : 0;
  const int spad_count = spad_info & 0x7F;
  int enabled = 0;
  for (int i = 0; i < 48; ++i) {
    if (i < first_spad || enabled == spad_count)
      spad_map[i / 8] &= ~(1 << (i % 8));
    else if (spad_map[i / 8] & (1 << (i % 8)))
      ++enabled;
  }
  for (int i = 0; i < 6; ++i)
    write_register(0xB0 + i, spad_map[i]);
}

void VL53L0XV2::wait_ready(std::uint8_t addr, std::uint8_t mask) {
  const std::int64_t start = esp_timer_get_time();
  while ((read_register(addr) & mask) == 0) {
    ESP_ERROR_CHECK(esp_timer_get_time() - start < 500000 ? ESP_OK : ESP_ERR_TIMEOUT);
    vTaskDelay(1);
  }
}

void VL53L0XV2::calibrate(std::uint8_t sequence, std::uint8_t start) {
  write_register(0x01, sequence);
  write_register(SYSRANGE_START, start);
  wait_ready(RESULT_INTERRUPT_STATUS, 0x07);
  write_register(SYSTEM_INTERRUPT_CLEAR, 0x01);
  write_register(SYSRANGE_START, 0x00);
}

bool VL53L0XV2::update() {
  std::uint8_t status{};
  if (read_register(RESULT_INTERRUPT_STATUS, &status, 1) != ESP_OK) {
    valid = false;
    return false;
  }
  if ((status & 0x07) == 0)
    return false;

  // Read status and distance together, then acknowledge this measurement.
  std::uint8_t result[12]{};
  valid = false;
  if (read_register(RESULT_RANGE_STATUS, result, sizeof(result)) != ESP_OK)
    return false;

  const std::uint8_t clear[]{SYSTEM_INTERRUPT_CLEAR, 0x01};
  if (i2c_master_transmit(distance_sensor, clear, sizeof(clear), I2C_TIMEOUT_MS) != ESP_OK)
    return false;
  if (((result[0] & 0x78) >> 3) != 11)
    return false;

  distance_mm = static_cast<std::uint16_t>((result[10] << 8) | result[11]);
  sample_time_us = esp_timer_get_time();
  valid = true;
  return true;
}

bool VL53L0XV2::is_valid(std::uint32_t max_age_ms) const {
  return valid && esp_timer_get_time() - sample_time_us <= static_cast<std::int64_t>(max_age_ms) * 1000;
}

std::uint16_t VL53L0XV2::get_dist_mm() {
  return is_valid() ? distance_mm : UINT16_MAX;
}

void VL53L0XV2::set_addr(std::uint8_t NEW_ADDR) {
  // Communicate on 0x29 before adress mod.
  write_register(ADDRS_REGISTER, NEW_ADDR);

  // remake device handle with new adress.
  i2c_device_config_t dist_config{};
  dist_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dist_config.device_address = NEW_ADDR;
  dist_config.scl_speed_hz = 100000;

  // remove old handle to prevent leak
  ESP_ERROR_CHECK(i2c_master_bus_rm_device(distance_sensor));
  distance_sensor = nullptr;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dist_config, &distance_sensor));
}

void VL53L0XV2::write_register(std::uint8_t addr, std::uint8_t value) {
  const std::uint8_t cmd[]{addr, value};
  ESP_ERROR_CHECK(i2c_master_transmit(distance_sensor, cmd, sizeof(cmd), I2C_TIMEOUT_MS));
}

void VL53L0XV2::driveXSHUT(uint32_t state) {
  ESP_ERROR_CHECK(gpio_set_level(XSHUT_PIN, state));
}

gpio_config_t VL53L0XV2::getConfig() {
  return conf;
}

std::uint8_t VL53L0XV2::read_register(std::uint8_t addr) {
  std::uint8_t value{};
  ESP_ERROR_CHECK(read_register(addr, &value, 1));
  return value;
}

esp_err_t VL53L0XV2::read_register(std::uint8_t addr, std::uint8_t *buffer, std::size_t len) {
  return i2c_master_transmit_receive(distance_sensor, &addr, 1, buffer, len, I2C_TIMEOUT_MS);
}
