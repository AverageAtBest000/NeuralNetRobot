#pragma once

#include "TB6612.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "soc/gpio_num.h"
#include <cstdint>
inline constexpr int PINS_A = 1;
inline constexpr int PINS_B = 0;
inline constexpr int FORWARD = 1;
inline constexpr int REVERSE = 0;
inline constexpr float TWO_PI = 6.2831853071795864769f;
inline constexpr uint32_t MAX_DUTY = (1U << 10) - 1;

/**
 * Wrapper class for GA12N20 motors
 */
class GA12N20 {

private:
  // Pin declarations
  const gpio_num_t IN1_PIN;
  const gpio_num_t IN2_PIN;
  const gpio_num_t PWM_PIN;
  const gpio_num_t ENCODER1_PIN;
  const gpio_num_t ENCODER2_PIN;
  const ledc_channel_t pwm_channel;
  gpio_config_t input_conf;
  gpio_config_t output_conf;
  ledc_timer_config_t timer;
  ledc_channel_config_t channel;

  // Velocity control declarations
  float velocity = 0.0f;
  float target_velocity = 0.0f;
  const float ticks_per_rotation;
  // Gains act on rad/s error and produce signed duty in [-1, 1].
  float kp = 0.1f;
  float ki = 0.05f;
  float kd = 0.0f;
  float integral_output = 0.0f;

  int64_t encoder_count = 0;
  portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

  int64_t previous_count = 0;
  int64_t previous_time_us = 0;
  bool has_previous_sample = false;

  /**@brief called when rising endge is detected in encoder A
   * @note calls a system interrupt
   * */
  static void encoderISR(void *arg);

  /**
   * @brief sets up system interrupt
   * @note gpio_install_isr_service() must be called in main
   * */
  void startEncoder();

  /**
   * @brief sets motor direction
   * @param direction - if FORWARD - AIN1 HIGH, AIN2 LOW, opposite if REVERSE
   * */
  void setDirection(int direction);

public:
  /*
   * @brief Constuctor for the motor class
   * @param driver - TB6612 instance. Used to initialize two input pins and one pwd pin
   * @param partition - PINS_A or PINS_B
   * @param ENCODER1_PIN - gpio number for first encoder wire
   * @param ENCODER2_PIN - gpio number for second encoder wire
   * @param ticks_per_rotation - encoder A rising edges per output shaft revolution,
   * including the gearbox ratio (not quadrature x4 counts); must be positive
   * @note Uses LEDC low-speed timer 0 at 20 kHz, channels 0 (A) and 1 (B).
   * Install the GPIO ISR service before construction. Configures GPIO/PWM and enables STBY.
   * */
  GA12N20(TB6612 driver, const int partition, const gpio_num_t ENCODER1_PIN, const gpio_num_t ENCODER2_PIN, float ticks_per_rotation);
  // Call periodically from one task; the first call establishes the sampling baseline.
  void run();

  // Nonnegative, finite gains; defaults are starting values requiring hardware tuning.
  void setPIDGains(float proportional, float integral, float derivative);

  /**
   * @brief sets desired velocity
   * @note only acted upon in the run() method
   * @param velocity - signed desired velocity in radians per second; zero stops PWM
   *
   * */
  void setVelocity(float velocity);

  /**
   * @brief counts an encoder A rising edge using encoder B for direction
   *
   * */
  void updateTicks();

  /**
   *@brief returns measured signed velocity in radians per second
   */
  float getVelocity() const;

  /**
   *@brief returns gpio config
   */
  gpio_config_t getInConfig() const;
  gpio_config_t getOutConfig() const;
  ledc_timer_config_t getLedcTimerConfig() const;
  ledc_channel_config_t getLedcChannelConfig() const;
};
