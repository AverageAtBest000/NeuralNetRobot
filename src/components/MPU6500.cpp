#include "driver/i2c_master.h"
#include "esp_err.h"
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
  MPU6500(i2c_master_bus_handle_t bus) {

    // MPU6500 config
    i2c_device_config_t imu_config{};
    imu_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    imu_config.device_address = ADDRS;
    imu_config.scl_speed_hz = 100000;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &imu_config, &imu));

    // wake MPU6500. Send signal to PWR_MGMT_1
    uint8_t wake_cmd[] = {0x6B, 0x00};

    ESP_ERROR_CHECK(i2c_master_transmit(imu, wake_cmd, sizeof(wake_cmd), 1000));
  }

  void read() {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(imu, &reg, sizeof(reg), reads,
                                                sizeof(reads), 1000));
  }

  /**
   * @brief Merges high and low bits for a 16 bit measurement
   */
  void merge_bits() {
    a_x = static_cast<std::uint16_t>(reads[0] << 8 | reads[1]);
    a_y = static_cast<std::uint16_t>(reads[2] << 8 | reads[3]);
    a_z = static_cast<std::uint16_t>(reads[4] << 8 | reads[5]);
  }

  /**
   * @brief Acceleration getters
   */
  std::uint8_t get_ax() { return a_x; }
  std::uint8_t get_ay() { return a_y; }
  std::uint8_t get_az() { return a_z; }
};
