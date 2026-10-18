
#include "driver/i2c_types.h"
#include <cstddef>
#include <cstdint>
class VL53L0XV2 {
private:
  static constexpr std::uint8_t ADDRS = 0x29;
  const std::uint8_t SYSRANGE_START = 0x00;
  const std::uint8_t RESULT_INTERUPT_STATUS = 0x13;
  const std::uint8_t RESULT_RANGE_STATUS = 0x13;
  const std::uint8_t DIST_RESULT = 0x1E;
  const std::uint8_t SYSTEM_INTERRUPT_CLEAR = 0x0B;

  std::uint16_t distance_mm = -1;
  i2c_master_dev_handle_t distance_sensor;

  bool is_ready();
  bool is_valid();
  void write_to_register(std::uint8_t cmd[], std::size_t len);
  void read_register(std::uint8_t read_addr, std::uint8_t *read_buffer, std::size_t len);

public:
  VL53L0XV2(i2c_master_bus_handle_t &bus);
  std::uint16_t get_dist_mm();
  void read();
  void update();
};
