#include "map.h"

#include <stdint.h>
#include <stdio.h>

void map_run(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) {
  uint8_t out = map(x, in_min, in_max, out_min, out_max);
  printf("map(%u, %u, %u, %u, %u) = %u\n", x, in_min, in_max, out_min, out_max, out);
}

void main() {
  map_run(49, 50, 100, 20, 40);
  map_run(50, 50, 100, 20, 40);
  map_run(51, 50, 100, 20, 40);
  map_run(99, 50, 100, 20, 40);
  map_run(100, 50, 100, 20, 40);
  map_run(101, 50, 100, 20, 40);
  map_run(75, 50, 100, 20, 40);

  map_run(0, 0, 255, 20, 40);
  map_run(127, 0, 255, 20, 40);
}
