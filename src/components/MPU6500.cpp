#include "driver/i2c_master.h"
#include <cstdint>
#include <cstdio>

class MPU6500 {
private:
  static constexpr uint8_t ADDRS = 0x68;
  i2c_master_dev_handle_t imu;

public:
  MPU6500(i2c_master_bus_handle_t bus) {
    i2c_device_config imu_config();

    ESP_ERROR_CHECK(i2c_master_bus_device(bus, &imu_config, &imu));
  }
};
