
#include "driver/i2c_types.h"
#include <cstdint>
class VL53L0XV2 {
private:
  static constexpr std::uint8_t ADDRS = 0x29;
  const std::uint8_t SYSRANGE_START = 0x00;
  const std::uint8_t RESULT_INTERUPT_STATUS = 0x13;
  const std::uint8_t RESULT_RANGE_STATUS = 0x13;
  const std::uint8_t DIST_RESULT = 0x1E;
  const std::uint8_t SYSTEM_INTERRUPT_CLEAR = 0x0B;
  i2c_master_dev_handle_t distance_sensor;

public:
  VL53L0XV2(i2c_master_bus_handle_t &bus);
};
