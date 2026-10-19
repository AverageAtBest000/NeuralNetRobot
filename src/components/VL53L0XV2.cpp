#include "VL53L0XV2.hpp"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "utils.hpp"
#include <cstddef>
#include <cstdint>
#include <sys/types.h>

VL53L0XV2::VL53L0XV2(i2c_master_bus_handle_t &bus) {

  i2c_device_config_t dist_config{};
  dist_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dist_config.device_address = ADDRS;
  dist_config.scl_speed_hz = 100000;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dist_config, &distance_sensor));

  std::uint8_t set_continuous[] = {SYSRANGE_START, 0x02};
  write_to_register(set_continuous, sizeof(set_continuous));
}

void VL53L0XV2::update() {
  if (!is_ready() || !is_valid())
    return;

  std::uint8_t distance_bits[2]{};
  read_register(DIST_RESULT, distance_bits, sizeof(distance_bits));

  distance_mm = static_cast<std::uint16_t>(static_cast<std::uint16_t>(distance_bits[0] << 8) | distance_bits[1]);

  std::uint8_t clear[2]{SYSTEM_INTERRUPT_CLEAR, 0x01};
  write_to_register(clear, sizeof(clear));
}

bool VL53L0XV2::is_ready() {
  std::uint8_t interrupt_status{};
  read_register(RESULT_INTERUPT_STATUS, &interrupt_status, sizeof(interrupt_status));
  if ((interrupt_status & 0x07) == 0)
    return false;

  return true;
}

bool VL53L0XV2::is_valid() {
  std::uint8_t range_status{};
  read_register(RESULT_RANGE_STATUS, &range_status, sizeof(range_status));
  uint8_t status_code = (range_status & 0x78) >> 3;
  if (status_code == 11)
    return true;

  return false;
}

void VL53L0XV2::write_to_register(std::uint8_t cmd[], std::size_t len) {
  ESP_ERROR_CHECK(i2c_master_transmit(distance_sensor, cmd, len, 1000));
}

void VL53L0XV2::read_register(std::uint8_t read_addr, std::uint8_t *read_buffer, std::size_t len) {
  ESP_ERROR_CHECK(i2c_master_transmit_receive(distance_sensor, &read_addr, 1, read_buffer, len, 1000));
}

std::uint16_t VL53L0XV2::get_dist_mm() {
  return distance_mm;
}
