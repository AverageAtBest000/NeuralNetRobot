#include "GA12N20.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include <cstddef>

GA12N20::GA12N20(TB6612 driver, const int partition, const gpio_num_t ENCODER1_PIN, const gpio_num_t ENCODER2_PIN) : IN1_PIN((A) ? driver.AIN1 : driver.BIN1), IN2_PIN((A) ? driver.AIN2 : driver.BIN2),
                                                                                                                     PWM_PIN((A) ? driver.PWMA : driver.PWMB), ENCODER1_PIN(ENCODER1_PIN), ENCODER2_PIN(ENCODER2_PIN), velocity(0.0f), encoder_count(0.0f) {
  conf = {
      .pin_bit_mask = (1ULL << ENCODER1_PIN) | (1ULL << ENCODER2_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_POSEDGE,
  };

  startEncoder();
}

void GA12N20::run() {
}

void GA12N20::setDirection(int direction) {
}

float GA12N20::getVelocity() const {
  return velocity;
}

void GA12N20::setVelocity(float velocity) {
  target_velocity = velocity;
}

void GA12N20::startEncoder() {
  ESP_ERROR_CHECK(gpio_set_intr_type(ENCODER1_PIN, GPIO_INTR_POSEDGE));
  ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER1_PIN, encoderISR, nullptr));
}

void GA12N20::encoderISR(void *arg) {
}

gpio_config_t GA12N20::getConfig() const {
  return conf;
}
