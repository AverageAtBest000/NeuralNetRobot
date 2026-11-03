#include "GA12N20.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include <cstddef>
#include <cstdint>

GA12N20::GA12N20(TB6612 driver, const int partition, const gpio_num_t ENCODER1_PIN, const gpio_num_t ENCODER2_PIN) : IN1_PIN((partition == PINS_A) ? driver.AIN1 : driver.BIN1), IN2_PIN((partition == PINS_A) ? driver.AIN2 : driver.BIN2),
                                                                                                                     PWM_PIN((partition == PINS_A) ? driver.PWMA : driver.PWMB), ENCODER1_PIN(ENCODER1_PIN), ENCODER2_PIN(ENCODER2_PIN) {
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
  portENTER_CRITICAL(&encoderMux);
  int64_t current_count = encoder_count;
  portEXIT_CRITICAL(&encoderMux);

  int64_t current_time_us = esp_timer_get_time();

  if (!has_previous_sample) {
    previous_count = current_count;
    previous_time_us = current_time_us;
    has_previous_sample = true;
    return;
  }

  float dt = (previous_time_us - current_time_us) / 1000000.0f;

  if (dt <= 0)
    return;

  velocity = (current_count - previous_count) / dt;

  previous_count = current_count;
  previous_time_us = current_time_us;
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
  ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER1_PIN, encoderISR, this));
}

void GA12N20::encoderISR(void *arg) {
  GA12N20 *motor = static_cast<GA12N20 *>(arg);
  motor->updateTicks();
}

void GA12N20::updateTicks() {
  portENTER_CRITICAL_ISR(&encoderMux);
  encoder_count += (gpio_get_level(ENCODER2_PIN) == 0) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

gpio_config_t GA12N20::getConfig() const {
  return conf;
}
