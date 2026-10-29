#include "TB6612.hpp"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#define A 0

class GA12N20 {

private:
  void setDirection();
  const gpio_num_t IN1_PIN;
  const gpio_num_t IN2_PIN;
  const gpio_num_t PWM_PIN;
  const gpio_num_t ENCODER1_PIN;
  const gpio_num_t ENCODER2_PIN;
  gpio_config_t conf;
  float velocity;
  float target_velocity;
  float encoder_count;

public:
  GA12N20(TB6612 driver, const gpio_num_t ENCODER1_PIN, const gpio_num_t ENCODER2_PIN);
  void run();
  void setVelocity(float velocity);
  float getVelocity() const;
  void setPWM(float pwm);
  gpio_config_t getConfig() const;
};
