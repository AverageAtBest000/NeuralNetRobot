#include "TB6612.hpp"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
inline constexpr int PINS_A = 1;
inline constexpr int PINS_B = 0;
inline constexpr int FORWARD = 1;
inline constexpr int REVERSE = 0;

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
  gpio_config_t conf;

  // Velocity control declarations
  float velocity;
  float target_velocity;
  float encoder_count;

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
   * @param partition - if 0, use pins A on the driver param, else use pins B
   * @param ENCODER1_PIN - gpio number for first encoder wire
   * @param ENCODER2_PIN - gpio number for second encoder wire
   * */
  GA12N20(TB6612 driver, const int partition, const gpio_num_t ENCODER1_PIN, const gpio_num_t ENCODER2_PIN);
  void run();

  /**
   * @brief sets desired velocity
   * @note only acted upon in the run() method
   * @param velocity - desired velocity
   *
   * */
  void setVelocity(float velocity);

  /**
   *@brief returns curent velocity in
   */
  float getVelocity() const;

  /**
   *@brief returns gpio config
   *@note config should always be applied before run() is called
   */
  gpio_config_t getConfig() const;
};
