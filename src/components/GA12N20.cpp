#include "GA12N20.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"
#include "soc/gpio_num.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

GA12N20::GA12N20(TB6612 driver, const int partition, const gpio_num_t ENCODER1_PIN, const gpio_num_t ENCODER2_PIN)
    : STBY(driver.STBY), IN1_PIN((partition == PINS_A) ? driver.AIN1 : driver.BIN1), IN2_PIN((partition == PINS_A) ? driver.AIN2 : driver.BIN2),
      PWM_PIN((partition == PINS_A) ? driver.PWMA : driver.PWMB), ENCODER1_PIN(ENCODER1_PIN), ENCODER2_PIN(ENCODER2_PIN),
      pwm_channel((partition == PINS_A) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1) {

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

  channel = {
      .gpio_num = PWM_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = pwm_channel,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
  };
}

void GA12N20::setDirection(int direction) {
  ESP_ERROR_CHECK(gpio_set_level(IN1_PIN, direction == FORWARD));
  ESP_ERROR_CHECK(gpio_set_level(IN2_PIN, direction == REVERSE));
}

void GA12N20::setDuty(float requested_duty) {
  requested_duty = std::clamp(requested_duty, -1.0f, 1.0f);

  const int direction = (requested_duty > 0.0f) ? FORWARD : REVERSE;
  if (direction != previous_direction) {
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, pwm_channel, 0));
    setDirection(direction);
    previous_direction = direction;
  }

  const uint32_t duty = static_cast<uint32_t>(std::lround(std::fabs(requested_duty) * MAX_DUTY));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, duty));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel));
}

void GA12N20::startEncoder() {
  pcnt_unit_config_t unit_cfg{};
  unit_cfg.low_limit = -30000;
  unit_cfg.high_limit = 30000;
  unit_cfg.flags.accum_count = true;
  pcnt_unit_handle_t unit = nullptr;
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

  pcnt_glitch_filter_config_t filter{};
  filter.max_glitch_ns = 1000;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));

  pcnt_chan_config_t a_cfg{};
  a_cfg.edge_gpio_num = ENCODER1_PIN;
  a_cfg.level_gpio_num = ENCODER2_PIN;
  pcnt_channel_handle_t chan_a = nullptr;
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &a_cfg, &chan_a));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  pcnt_chan_config_t b_cfg{};
  b_cfg.edge_gpio_num = ENCODER2_PIN;
  b_cfg.level_gpio_num = ENCODER1_PIN;
  pcnt_channel_handle_t chan_b = nullptr;
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &b_cfg, &chan_b));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, unit_cfg.low_limit));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, unit_cfg.high_limit));

  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
  encoder_unit = unit;
}

gpio_config_t GA12N20::getInConfig() const {
  return input_conf;
}

gpio_config_t GA12N20::getOutConfig() const {
  return output_conf;
}

int64_t GA12N20::getEncoderTicks() {
  int count;
  ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_unit, &count));
  return count;
}

void GA12N20::resetEncoder() {
  ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
}

ledc_timer_config_t GA12N20::getLedcTimerConfig() {
  return ledc_timer_config_t{
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = 20000,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = false,
  };
}

ledc_channel_config_t GA12N20::getLedcChannelConfig() const {
  return channel;
}
