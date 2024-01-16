#pragma once

#include <cstdint>

struct TouchpadData {
  uint8_t num_touch_points = 0;
  uint16_t x = 0;
  uint16_t y = 0;
  uint8_t btn_state = 0;
};

// operator == for TouchpadData
[[maybe_unused]] static bool operator==(const TouchpadData& lhs, const TouchpadData& rhs) {
  return lhs.num_touch_points == rhs.num_touch_points &&
         lhs.x == rhs.x &&
         lhs.y == rhs.y &&
         lhs.btn_state == rhs.btn_state;
}
