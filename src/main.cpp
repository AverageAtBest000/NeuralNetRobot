// #include "MPU6500.hpp"
// #include "driver/i2c_master.h"
// #include "driver/i2c_types.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "hal/i2c_types.h"
// #include "soc/clk_tree_defs.h"
// #include <iostream>
// #include <stdio.h>
//
// void set_bus_conf(i2c_master_bus_config_t &bus_conf);
//
// extern "C" void app_main(void) {
//
//   i2c_master_bus_config_t bus_conf{};
//   set_bus_conf(bus_conf);
//
//   i2c_master_bus_handle_t bus{};
//   ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus));
//
//   MPU6500 imu{bus};
//
//   while (true) {
//     imu.update();
//
//     std::cout << "ax: " << imu.get_ax() << '\n';
//     std::cout << "ay: " << imu.get_ay() << '\n';
//     std::cout << "az: " << imu.get_az() << '\n';
//
//     std::cout << "gx: " << imu.get_gx() << '\n';
//     std::cout << "gy: " << imu.get_gy() << '\n';
//     std::cout << "gz: " << imu.get_gz() << '\n';
//
//     vTaskDelay(pdMS_TO_TICKS(100));
//   }
// }
//
// void set_bus_conf(i2c_master_bus_config_t &bus_conf) {
//   bus_conf.i2c_port = I2C_NUM_0;
//   bus_conf.sda_io_num = GPIO_NUM_21;
//   bus_conf.scl_io_num = GPIO_NUM_22;
//   bus_conf.clk_source = I2C_CLK_SRC_DEFAULT;
//   bus_conf.glitch_ignore_cnt = 7;
//   bus_conf.flags.enable_internal_pullup = true;
// }

#include <cstdint>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr gpio_num_t IN1 = GPIO_NUM_25;
static constexpr gpio_num_t IN2 = GPIO_NUM_26;
static constexpr gpio_num_t PWM = GPIO_NUM_27;
static constexpr gpio_num_t STBY = GPIO_NUM_33;

static constexpr auto MODE = LEDC_LOW_SPEED_MODE;
static constexpr auto CHANNEL = LEDC_CHANNEL_0;

static constexpr unsigned TEST_PERCENT = 30;
static_assert(TEST_PERCENT > 0 && TEST_PERCENT <= 70,
              "Keep the test duty between 1% and 70%.");

static const char *TAG = "motor_test";

static void set_pwm(unsigned percent) {
  // 10-bit PWM: duty values from 0 through 1023.
  const uint32_t duty = (1024U * percent) / 100U;

  ESP_ERROR_CHECK(ledc_set_duty(MODE, CHANNEL, duty));
  ESP_ERROR_CHECK(ledc_update_duty(MODE, CHANNEL));
}

static void stop_motor() {
  // Disable both driver channels; motor coasts.
  ESP_ERROR_CHECK(gpio_set_level(STBY, 0));
  set_pwm(0);
}

static void run_motor(bool reverse) {
  // Change direction while the driver is disabled.
  stop_motor();

  ESP_ERROR_CHECK(gpio_set_level(IN1, reverse ? 0 : 1));
  ESP_ERROR_CHECK(gpio_set_level(IN2, reverse ? 1 : 0));

  set_pwm(TEST_PERCENT);
  ESP_ERROR_CHECK(gpio_set_level(STBY, 1));
}

extern "C" void app_main() {
  // Configure the direction and enable pins.
  gpio_config_t pins{};
  pins.pin_bit_mask =
      (1ULL << IN1) | (1ULL << IN2) | (1ULL << STBY);
  pins.mode = GPIO_MODE_OUTPUT;
  pins.pull_up_en = GPIO_PULLUP_DISABLE;
  pins.pull_down_en = GPIO_PULLDOWN_ENABLE;
  pins.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&pins));

  ESP_ERROR_CHECK(gpio_set_level(STBY, 0));
  ESP_ERROR_CHECK(gpio_set_level(IN1, 0));
  ESP_ERROR_CHECK(gpio_set_level(IN2, 0));

  // Generate 20 kHz PWM with 10-bit resolution.
  ledc_timer_config_t timer{};
  timer.speed_mode = MODE;
  timer.duty_resolution = LEDC_TIMER_10_BIT;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = 20000;
  timer.clk_cfg = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer));

  ledc_channel_config_t channel{};
  channel.gpio_num = PWM;
  channel.speed_mode = MODE;
  channel.channel = CHANNEL;
  channel.intr_type = LEDC_INTR_DISABLE;
  channel.timer_sel = LEDC_TIMER_0;
  channel.duty = 0;
  channel.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&channel));

  ESP_LOGI(TAG, "Test begins in 5 seconds. Keep wheel clear.");
  vTaskDelay(pdMS_TO_TICKS(5000));

  ESP_LOGI(TAG, "Direction 1: %u%% PWM", TEST_PERCENT);
  run_motor(false);
  vTaskDelay(pdMS_TO_TICKS(2000));

  ESP_LOGI(TAG, "Coasting for 2 seconds");
  stop_motor();
  vTaskDelay(pdMS_TO_TICKS(2000));

  ESP_LOGI(TAG, "Direction 2: %u%% PWM", TEST_PERCENT);
  run_motor(true);
  vTaskDelay(pdMS_TO_TICKS(2000));

  stop_motor();
  ESP_LOGI(TAG, "Done. Driver disabled. Reset to repeat.");
}
