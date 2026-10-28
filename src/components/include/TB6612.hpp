
#include "driver/gpio.h"
#include "soc/gpio_num.h"

class TB6612 {
public:
  const gpio_num_t AIN1;
  const gpio_num_t AIN2;
  const gpio_num_t PWMA;
  const gpio_num_t BIN1;
  const gpio_num_t BIN2;
  const gpio_num_t PWMB;
  const gpio_num_t STBY;

  TB6612(gpio_num_t AIN1, gpio_num_t AIN2, gpio_num_t PWMA, gpio_num_t BIN1, gpio_num_t BIN2, gpio_num_t PWMB, gpio_num_t STBY) : AIN1(AIN1), AIN2(AIN2), PWMA(PWMA), BIN1(BIN1), BIN2(BIN2), PWMB(PWMB), STBY(STBY) {};
};
