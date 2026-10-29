#include "GA12N20.hpp"
#include "driver/gpio.h"
#include "soc/gpio_num.h"

GA12N20::GA12N20(TB6612 driver, const gpio_num_t ENCODER1_PIN, const gpio_num_t ENCODER2_PIN) : IN1_PIN((A) ? driver.AIN1 : driver.BIN1), IN2_PIN((A) ? driver.AIN2 : driver.BIN2),
                                                                                                PWM_PIN((A) ? driver.PWMA : driver.PWMB), ENCODER1_PIN(ENCODER1_PIN), ENCODER2_PIN(ENCODER2_PIN), velocity(0.0f), encoder_count(0.0f) {
  conf = {
      .pin_bit_mask = (1ULL << ENCODER1_PIN) | (1ULL << ENCODER2_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_POSEDGE,
  };
}

void GA12N20::run() {
}

float GA12N20::getVelocity() const {
  return velocity;
}

void GA12N20::setPWM(float pwm) {
}

void GA12N20::setVelocity(float velocity) {
}

gpio_config_t GA12N20::getConfig() const {
  return conf;
}
