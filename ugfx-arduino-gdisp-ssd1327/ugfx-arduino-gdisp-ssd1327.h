#ifndef UGFX_ARDUINO_SSD1327_H
#define UGFX_ARDUINO_SSD1327_H

#include <stdint.h>

typedef struct {
  const uint8_t reset;
  const uint8_t cs;
  const uint8_t dc;
} SSD1327_pins_t;

extern const SSD1327_pins_t SSD1327_pins;

#endif // UGFX_ARDUINO_SSD1327_H
