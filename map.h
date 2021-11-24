#ifndef MAP_H
#define MAP_H
#include <stdint.h>

// https://www.arduino.cc/reference/en/language/functions/math/map/

uint8_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif
