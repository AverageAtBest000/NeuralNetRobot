#pragma once

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
};
