#pragma once

#include "esp_log.h"

namespace uils {
inline void raise_warning(const char *source, const char *warning) {
  ESP_LOGW(source, "warning: %s", warning);
}
} // namespace uils
