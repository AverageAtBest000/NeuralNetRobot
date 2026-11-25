#pragma once

#include "TB6612.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "soc/gpio_num.h"
#include <cstdint>
inline constexpr int PINS_A = 1;
inline constexpr int PINS_B = 0;
inline constexpr int FORWARD = 1;
inline constexpr int REVERSE = 0;
inline constexpr uint32_t MAX_DUTY = (1U << 10) - 1;

/**
 * @brief Controls one GA12N20 motor using a TB6612 driver.
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

  int previous_direction = -1;

  // pcnt encoder handle
  pcnt_unit_handle_t encoder_unit;

  /**
   * @brief Sets the direction pins for the selected TB6612 channel.
   * @param direction - FORWARD sets IN1 HIGH and IN2 LOW; REVERSE sets the opposite.
   * @note setDuty() stops PWM before changing direction.
   */
  void setDirection(int direction);

public:
  /**
   * @brief Stores motor pins and prepares GPIO/PWM configuration.
   * @param driver - TB6612 pin mapping providing direction, PWM, and shared STBY pins.
   * @param partition - PINS_A selects channel A; PINS_B selects channel B.
   * @param ENCODER1_PIN - GPIO for encoder A.
   * @param ENCODER2_PIN - GPIO for encoder B.
   * @note Uses LEDC low-speed timer 0 at 20 kHz, channels 0 (A) and 1 (B).
   * @note Construction does not configure hardware or enable STBY. Apply the returned
   * configurations in app_main(), configuring the shared timer once. Keep STBY LOW
   * until both motors' direction pins are LOW and their PWM channels have zero duty.
   */
  GA12N20(TB6612 driver, const int partition, const gpio_num_t ENCODER1_PIN, const gpio_num_t ENCODER2_PIN);

  void startEncoder();

  /**
   * @brief Returns the latest measured encoder tick count
   */
  int64_t getEncoderTicks();

  /**
   * @brief Resets encoder tick count to zero
   */
  void resetEncoder();

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

  void setDuty(float duty);
};
