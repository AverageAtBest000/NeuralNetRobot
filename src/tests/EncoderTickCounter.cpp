#include "components/include/GA12N20.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
#include <inttypes.h>

/**
 * @brief Prints the encoder count of a motor.
 * @note Uses GPIO0. Do not run this test while using GPIO 0 as a VLX XSHUT pin.
 */

int CountEncoderTicks(GA12N20 &motor) {
  constexpr gpio_num_t BOOT_BUTTON_PIN = GPIO_NUM_0;
  gpio_config_t button_config{};
  button_config.pin_bit_mask = 1ULL << BOOT_BUTTON_PIN;
  button_config.mode = GPIO_MODE_INPUT;
  button_config.pull_up_en = GPIO_PULLUP_ENABLE;
  button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  button_config.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&button_config));

  motor.resetEncoder();

  while (gpio_get_level(BOOT_BUTTON_PIN) != 0) {
    printf("Encoder ticks: %" PRId64 "\n", motor.getEncoderTicks());
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  return 0;
}
