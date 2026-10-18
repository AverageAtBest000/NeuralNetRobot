#include "VL53L0XV2.hpp"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"

VL53L0XV2::VL53L0XV2(i2c_master_bus_handle_t &bus) {

  i2c_device_config_t dist_config{};
  dist_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dist_config.device_address = ADDRS;
  dist_config.scl_speed_hz = 100000;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dist_config, &distance_sensor));

  std::uint8_t set_continuous[] = {SYSRANGE_START, 0x02};
  ESP_ERROR_CHECK(i2c_master_transmit(distance_sensor, set_continuous, sizeof(set_continuous), 1000));
}
