#include "GA12N20.hpp"
#include "MPU6500.hpp"
#include "TB6612.hpp"
#include "VL53L0XV2.hpp"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "hal/ledc_types.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_num.h"
#include "tests/include/EncoderTickCounter.hpp"
#include <cstdint>
#include <iostream>
#include <sys/reent.h>

enum class Mode {
  ENCODER_TEST,
  RUN
};

constexpr Mode MODE = Mode::ENCODER_TEST;

void set_bus_conf(i2c_master_bus_config_t &bus_conf);
void set_VLX_adress(VL53L0XV2 &front, VL53L0XV2 &right, VL53L0XV2 &left);

constexpr std::uint16_t FRONT_SENSOR_ADDRESS = 0x30;
constexpr std::uint16_t RIGHT_SENSOR_ADDRESS = 0x31;
constexpr std::uint16_t LEFT_SENSOR_ADDRESS = 0x32;

constexpr gpio_num_t AIN1 = GPIO_NUM_19;
constexpr gpio_num_t AINrightMotor2 = GPIO_NUM_18;
constexpr gpio_num_t PWMA = GPIO_NUM_5;
constexpr gpio_num_t RIGHT_MOTOR_ENCODER1_PIN = GPIO_NUM_17;
constexpr gpio_num_t RIGHT_MOTOR_ENCODER2_PIN = GPIO_NUM_16;

constexpr gpio_num_t BIN1 = GPIO_NUM_32;
constexpr gpio_num_t BIN2 = GPIO_NUM_33;
constexpr gpio_num_t PWMB = GPIO_NUM_25;
constexpr gpio_num_t LEFT_MOTOR_ENCODER1_PIN = GPIO_NUM_26;
constexpr gpio_num_t LEFT_MOTOR_ENCODER2_PIN = GPIO_NUM_27;

constexpr gpio_num_t STBY = GPIO_NUM_23;
constexpr float motor_ticks_per_rotation = 0.0f;

constexpr gpio_num_t FRONT_DIST_XSHUT = GPIO_NUM_2;
constexpr gpio_num_t RIGHT_DIST_XSHUT = GPIO_NUM_0;
constexpr gpio_num_t LEFT_DIST_XSHUT = GPIO_NUM_15;

extern "C" void app_main(void) {

  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  i2c_master_bus_config_t bus_conf{};
  set_bus_conf(bus_conf);

  i2c_master_bus_handle_t bus{};
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus));

  constexpr TB6612 driver{AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY};

  MPU6500 imu{bus};
  ESP_ERROR_CHECK(imu.calibrate_gyro() ? ESP_OK : ESP_ERR_INVALID_RESPONSE);

  VL53L0XV2 front_distance_sensor{bus, FRONT_DIST_XSHUT};
  VL53L0XV2 right_distance_sensor{bus, RIGHT_DIST_XSHUT};
  VL53L0XV2 left_distance_sensor{bus, LEFT_DIST_XSHUT};

  auto front_conf = front_distance_sensor.getConfig();
  auto right_conf = right_distance_sensor.getConfig();
  auto left_conf = left_distance_sensor.getConfig();
  ESP_ERROR_CHECK(gpio_config(&front_conf));
  ESP_ERROR_CHECK(gpio_config(&right_conf));
  ESP_ERROR_CHECK(gpio_config(&left_conf));

  set_VLX_adress(front_distance_sensor, right_distance_sensor, left_distance_sensor);

  front_distance_sensor.init();
  right_distance_sensor.init();
  left_distance_sensor.init();

  // Set the motors' STBY LOW before initialization
  gpio_set_direction(driver.STBY, GPIO_MODE_OUTPUT);
  ESP_ERROR_CHECK(gpio_set_level(driver.STBY, 0));

  GA12N20 rightMotor(driver, PINS_A, RIGHT_MOTOR_ENCODER1_PIN, RIGHT_MOTOR_ENCODER2_PIN, motor_ticks_per_rotation);
  GA12N20 leftMotor(driver, PINS_B, LEFT_MOTOR_ENCODER1_PIN, LEFT_MOTOR_ENCODER2_PIN, motor_ticks_per_rotation);

  auto right_input_conf = rightMotor.getInConfig();
  auto left_input_conf = leftMotor.getInConfig();
  ESP_ERROR_CHECK(gpio_config(&right_input_conf));
  ESP_ERROR_CHECK(gpio_config(&left_input_conf));

  auto right_output_conf = rightMotor.getOutConfig();
  auto left_output_conf = leftMotor.getOutConfig();
  ESP_ERROR_CHECK(gpio_config(&right_output_conf));
  ESP_ERROR_CHECK(gpio_config(&left_output_conf));

  auto timer_conf = GA12N20::getLedcTimerConfig();
  ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

  auto right_channel_conf = rightMotor.getLedcChannelConfig();
  auto left_channel_conf = leftMotor.getLedcChannelConfig();
  ESP_ERROR_CHECK(ledc_channel_config(&right_channel_conf));
  ESP_ERROR_CHECK(ledc_channel_config(&left_channel_conf));

  rightMotor.startEncoder();
  leftMotor.startEncoder();

  switch (MODE) {
  case Mode::ENCODER_TEST:
    ESP_ERROR_CHECK(gpio_set_level(driver.STBY, 0));
    CountEncoderTicks(leftMotor);
    break;
  case Mode::RUN:
    break;
  }
}

void set_VLX_adress(VL53L0XV2 &front, VL53L0XV2 &right, VL53L0XV2 &left) {

  front.driveXSHUT(0);
  right.driveXSHUT(0);
  left.driveXSHUT(0);

  front.driveXSHUT(1);
  vTaskDelay(pdMS_TO_TICKS(20));
  front.set_addr(FRONT_SENSOR_ADDRESS);

  right.driveXSHUT(1);
  vTaskDelay(pdMS_TO_TICKS(20));
  right.set_addr(RIGHT_SENSOR_ADDRESS);

  left.driveXSHUT(1);
  vTaskDelay(pdMS_TO_TICKS(20));
  left.set_addr(LEFT_SENSOR_ADDRESS);
}

void set_bus_conf(i2c_master_bus_config_t &bus_conf) {
  bus_conf.i2c_port = I2C_NUM_0;
  bus_conf.sda_io_num = GPIO_NUM_21;
  bus_conf.scl_io_num = GPIO_NUM_22;
  bus_conf.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_conf.glitch_ignore_cnt = 7;
  bus_conf.flags.enable_internal_pullup = true;
}
