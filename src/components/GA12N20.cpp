#include "GA12N20.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

GA12N20::GA12N20(TB6612 driver, const int partition, const gpio_num_t ENCODER1_PIN, const gpio_num_t ENCODER2_PIN, float ticks_per_rotation)
    : IN1_PIN((partition == PINS_A) ? driver.AIN1 : driver.BIN1), IN2_PIN((partition == PINS_A) ? driver.AIN2 : driver.BIN2),
      PWM_PIN((partition == PINS_A) ? driver.PWMA : driver.PWMB), ENCODER1_PIN(ENCODER1_PIN), ENCODER2_PIN(ENCODER2_PIN),
      pwm_channel((partition == PINS_A) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1), ticks_per_rotation(ticks_per_rotation) {

  input_conf = {
      .pin_bit_mask = (1ULL << ENCODER1_PIN) | (1ULL << ENCODER2_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  output_conf = {
      .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) | (1ULL << driver.STBY),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = 20000,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = false,
  };

  channel = {
      .gpio_num = PWM_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = pwm_channel,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
  };

  ESP_ERROR_CHECK(gpio_set_level(driver.STBY, 1));
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
    ESP_ERROR_CHECK(gpio_set_level(IN1_PIN, 0));
    ESP_ERROR_CHECK(gpio_set_level(IN2_PIN, 0));
    return;
  }

  float dt = (current_time_us - previous_time_us) / 1000000.0f;

  if (dt <= 0)
    return;

  const float previous_velocity = velocity;
  velocity = (current_count - previous_count) * (TWO_PI / ticks_per_rotation) / dt;

  float velocity_error = target_velocity - velocity;

  const float derivative = -(velocity - previous_velocity) / dt;
  const float candidate_integral = std::clamp(integral_output + ki * velocity_error * dt, -1.0f, 1.0f);
  const float candidate_output = kp * velocity_error + candidate_integral + kd * derivative;

  // Only integrate while unsaturated, or when the error unwinds saturation.
  if ((candidate_output >= -1.0f && candidate_output <= 1.0f) || (candidate_output > 1.0f && velocity_error < 0.0f) || (candidate_output < -1.0f && velocity_error > 0.0f)) {
    integral_output = candidate_integral;
  }
  float output = std::clamp(kp * velocity_error + integral_output + kd * derivative, -1.0f, 1.0f);
  if (target_velocity == 0.0f) {
    integral_output = 0.0f;
    output = 0.0f;
  }

  // Disable PWM before changing direction, then apply the requested duty.
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, 0));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel));
  setDirection(output >= 0.0f ? FORWARD : REVERSE);
  const uint32_t duty = static_cast<uint32_t>(std::lround(std::fabs(output) * MAX_DUTY));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, duty));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel));

  previous_count = current_count;
  previous_time_us = current_time_us;
}

void GA12N20::setDirection(int direction) {
  ESP_ERROR_CHECK(gpio_set_level(IN1_PIN, direction == FORWARD));
  ESP_ERROR_CHECK(gpio_set_level(IN2_PIN, direction == REVERSE));
}

void GA12N20::setPIDGains(float proportional, float integral, float derivative) {
  kp = proportional;
  ki = integral;
  kd = derivative;
  integral_output = 0.0f;
}

float GA12N20::getVelocity() const {
  return velocity;
}

void GA12N20::setVelocity(float velocity) {

  if (velocity == 0.0f || (velocity > 0.0f) != (target_velocity > 0.0f)) {
    integral_output = 0.0f;
  }
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

gpio_config_t GA12N20::getInConfig() const {
  return input_conf;
}

gpio_config_t GA12N20::getOutConfig() const {
  return output_conf;
}

ledc_timer_config_t GA12N20::getLedcTimerConfig() const {
  return timer;
}

ledc_channel_config_t GA12N20::getLedcChannelConfig() const {
  return channel;
}
