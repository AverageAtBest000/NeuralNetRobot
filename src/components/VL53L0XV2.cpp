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

bool VL53L0XV2::is_ready() {
  std::uint8_t interrupt_status{};
  read_register(RESULT_INTERUPT_STATUS, &interrupt_status, sizeof(interrupt_status));
  if ((interrupt_status & 0x07) != 0)
    return false;

  return true;
}

bool VL53L0XV2::is_valid() {
  std::uint8_t range_status{};
  read_register(RESULT_RANGE_STATUS, &range_status, sizeof(range_status));
  uint8_t status_code = (range_status & 0x78) >> 3;
  if (status_code == 0)
    return true;
  else if (status_code == 2)
    uils::raise_warning("VL53L0XV2", "singnal failiure");
  else if (status_code == 3)
    uils::raise_warning("VL53L0XV2", "minimum range failiure");
  else if (status_code == 4)
    uils::raise_warning("VL53L0XV2", "phase failiure");
  else if (status_code == 5)
    uils::raise_warning("VL53L0XV2", "hardware failiure");
  else if (status_code == 11)
    uils::raise_warning("VL53L0XV2", "no usable status");

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
