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
 * @brief Controls one GA12N20 motor using encoder feedback and a TB6612 driver.
 * @note The application applies GPIO/PWM configuration and controls the shared STBY pin.
 * Call task-side methods from one owning task or provide external synchronization.
 */
class GA12N20 {

private:
  // Pin declarations
  const gpio_num_t STBY;
  const gpio_num_t IN1_PIN;
  const gpio_num_t IN2_PIN;
  const gpio_num_t PWM_PIN;
  const gpio_num_t ENCODER1_PIN;
  const gpio_num_t ENCODER2_PIN;
  const ledc_channel_t pwm_channel;
  gpio_config_t input_conf;
  gpio_config_t output_conf;
  ledc_channel_config_t channel;

  // Velocity control declarations
  float velocity = 0.0f;
  float target_velocity = 0.0f;
  const float ticks_per_rotation;
  int previous_direction = -1;
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

  /**
   * @brief Handles a rising edge on encoder A.
   * @param arg - Pointer to the GA12N20 instance registered by startEncoder().
   * @note Called by the GPIO ISR service; forwards the edge to updateTicks().
   */
  static void encoderISR(void *arg);

  /**
   * @brief Sets the direction pins for the selected TB6612 channel.
   * @param direction - FORWARD sets IN1 HIGH and IN2 LOW; REVERSE sets the opposite.
   * @note run() stops PWM before changing direction.
   */
  void setDirection(int direction);

public:
  /**
   * @brief Stores motor pins and prepares GPIO/PWM configuration.
   * @param driver - TB6612 pin mapping providing direction, PWM, and shared STBY pins.
   * @param partition - PINS_A selects channel A; PINS_B selects channel B.
   * @param ENCODER1_PIN - GPIO for encoder A, whose rising edges are counted.
   * @param ENCODER2_PIN - GPIO for encoder B, sampled to determine rotation direction.
   * @param ticks_per_rotation - Encoder A rising edges per output shaft revolution,
   * including the gearbox ratio (not quadrature x4 counts); must be positive and finite.
   * @note Uses LEDC low-speed timer 0 at 20 kHz, channels 0 (A) and 1 (B).
   * @note Construction does not configure hardware or enable STBY. Apply the returned
   * configurations in app_main(), configuring the shared timer once. Keep STBY LOW
   * until both motors' direction pins are LOW and their PWM channels have zero duty.
   */
  GA12N20(TB6612 driver, const int partition, const gpio_num_t ENCODER1_PIN, const gpio_num_t ENCODER2_PIN, float ticks_per_rotation);

  /**
   * @brief Updates measured velocity and applies PID control to the PWM output.
   * @note Call periodically after applying GPIO/PWM configuration and starting the encoder.
   * The application must set the shared STBY pin HIGH to enable motor operation.
   * @note The first call establishes the sampling baseline, sets both direction pins LOW,
   * and returns without updating PWM. Later calls apply the requested velocity.
   */
  void run();

  /**
   * @brief Sets PID gains and clears the accumulated integral output.
   * @param proportional - Proportional gain; must be nonnegative and finite.
   * @param integral - Integral gain; must be nonnegative and finite.
   * @param derivative - Gain applied to the negative measured velocity derivative;
   * must be nonnegative and finite.
   * @note Gains convert rad/s error, its integral, and the measured derivative into
   * signed duty in [-1, 1]. The caller validates gains and tunes them for the hardware.
   */
  void setPIDGains(float proportional, float integral, float derivative);

  /**
   * @brief Stores the desired motor velocity for subsequent control updates.
   * @param velocity - Finite, signed target velocity in radians per second.
   * Zero requests zero PWM duty.
   * @note Does not change hardware immediately. run() applies the target after its
   * first baseline call. A zero target or a change of target sign clears the integral output.
   */
  void setVelocity(float velocity);

  /**
   * @brief Records one rising edge on encoder A using encoder B for direction.
   * @note Called in ISR context by encoderISR(). Encoder B LOW increments the count;
   * HIGH decrements it. Uses an ISR critical section to protect the count.
   */
  void updateTicks();

  /**
   * @brief Returns the latest measured motor velocity.
   * @return Signed velocity in radians per second, initially zero.
   * @note Updated by run() after its first baseline call; this getter does not sample the encoder.
   */
  float getVelocity() const;

  /**
   * @brief Registers the encoder A rising-edge interrupt handler.
   * @note Apply the encoder GPIO configuration and install the GPIO ISR service with
   * gpio_install_isr_service() before calling this method.
   * @note The motor object must remain at the same address and alive while its handler
   * is registered. The class does not automatically unregister the handler.
   */
  void startEncoder();

  /**
   * @brief Returns the GPIO configuration for both encoder inputs.
   * @return Input configuration with pull-ups enabled and interrupts disabled.
   * @note Apply with gpio_config() before calling startEncoder().
   */
  gpio_config_t getInConfig() const;

  /**
   * @brief Returns the GPIO configuration for the direction pins and shared STBY pin.
   * @return Output configuration for IN1, IN2, and STBY; excludes the PWM pin.
   * @note Apply with gpio_config() and explicitly initialize output levels in app_main().
   */
  gpio_config_t getOutConfig() const;

  /**
   * @brief Returns the LEDC timer configuration shared by both motors.
   * @return Low-speed timer 0 configuration at 20 kHz with 10-bit duty resolution.
   * @note Apply once with ledc_timer_config() before configuring either PWM channel.
   */
  static ledc_timer_config_t getLedcTimerConfig();

  /**
   * @brief Returns the LEDC channel configuration for this motor's PWM pin.
   * @return Channel 0 for PINS_A or channel 1 for PINS_B, using timer 0 and zero duty.
   * @note Apply with ledc_channel_config() after configuring the shared timer.
   */
  ledc_channel_config_t getLedcChannelConfig() const;
};
