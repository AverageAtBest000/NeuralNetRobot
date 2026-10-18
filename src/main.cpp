#include "MPU6500.hpp"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "soc/clk_tree_defs.h"
#include <stdio.h>

void set_bus_conf(i2c_master_bus_config_t &bus_conf);

extern "C" void app_main(void) {

  i2c_master_bus_config_t bus_conf{};
  set_bus_conf(bus_conf);

  i2c_master_bus_handle_t bus{};
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus));

  MPU6500 imu{bus};
}

void set_bus_conf(i2c_master_bus_config_t &bus_conf) {
  bus_conf.i2c_port = I2C_NUM_0;
  bus_conf.sda_io_num = GPIO_NUM_21;
  bus_conf.scl_io_num = GPIO_NUM_22;
  bus_conf.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_conf.glitch_ignore_cnt = 7;
  bus_conf.flags.enable_internal_pullup = true;
}
