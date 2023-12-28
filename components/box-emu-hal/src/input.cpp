#include <mutex>

#include <driver/gpio.h>

#include "timer.hpp"
#include "touchpad_input.hpp"
#include "keypad_input.hpp"

#include "input.h"
#include "box_emu_hal.hpp"
#include "hal_i2c.hpp"

using namespace std::chrono_literals;
using namespace box_hal;

struct TouchpadData {
  uint8_t num_touch_points = 0;
  uint16_t x = 0;
  uint16_t y = 0;
  uint8_t btn_state = 0;
};

static std::shared_ptr<InputDriver> input_driver;
static std::shared_ptr<TouchDriver> touch_driver;
static std::shared_ptr<espp::TouchpadInput> touchpad;
static std::shared_ptr<espp::KeypadInput> keypad;
static std::shared_ptr<espp::Timer> input_timer;
static struct InputState gamepad_state;
static std::mutex gamepad_state_mutex;
static TouchpadData touchpad_data;
static std::mutex touchpad_data_mutex;

/**
 * Touch Controller configuration
 */
void touchpad_read(uint8_t* num_touch_points, uint16_t* x, uint16_t* y, uint8_t* btn_state) {
  std::lock_guard<std::mutex> lock(touchpad_data_mutex);
  *num_touch_points = touchpad_data.num_touch_points;
  *x = touchpad_data.x;
  *y = touchpad_data.y;
  *btn_state = touchpad_data.btn_state;
}

void keypad_read(bool *up, bool *down, bool *left, bool *right, bool *enter, bool *escape) {
  InputState state;
  get_input_state(&state);
  *up = state.up;
  *down = state.down;
  *left = state.left;
  *right = state.right;

  *enter = state.a || state.start;
  *escape = state.b || state.select;
}

void update_touchpad_input() {
  // get the latest data from the device
  std::error_code ec;
  bool new_data = touch_driver->update(ec);
  if (ec) {
    fmt::print("error updating touch_driver: {}\n", ec.message());
    std::lock_guard<std::mutex> lock(touchpad_data_mutex);
    touchpad_data = {};
    return;
  }
  if (!new_data) {
    std::lock_guard<std::mutex> lock(touchpad_data_mutex);
    touchpad_data = {};
    return;
  }
  // get the latest data from the touchpad
  TouchpadData temp_data;
  touch_driver->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
  temp_data.btn_state = touch_driver->get_home_button_state();
  // update the touchpad data
  std::lock_guard<std::mutex> lock(touchpad_data_mutex);
  touchpad_data = temp_data;
}

void update_gamepad_input() {
  static bool can_read_input = true;
  bool is_select_pressed = false;
  bool is_start_pressed = false;
  bool is_up_pressed = false;
  bool is_down_pressed = false;
  bool is_left_pressed = false;
  bool is_right_pressed = false;
  bool is_a_pressed = false;
  bool is_b_pressed = false;
  bool is_x_pressed = false;
  bool is_y_pressed = false;
  if (!input_driver) {
    fmt::print("cannot get input state: input driver not initialized properly!\n");
    return;
  }
  if (!can_read_input) {
    return;
  }
  // pins are active low
  // start, select = A0, A1
  std::error_code ec;
  auto pins = input_driver->get_pins(ec);
  if (ec) {
    fmt::print("error getting pins: {}\n", ec.message());
    can_read_input = false;
    return;
  }
  is_start_pressed = !(pins & START_PIN);
  is_select_pressed = !(pins & SELECT_PIN);
  is_up_pressed = !(pins & UP_PIN);
  is_down_pressed = !(pins & DOWN_PIN);
  is_left_pressed = !(pins & LEFT_PIN);
  is_right_pressed = !(pins & RIGHT_PIN);
  is_a_pressed = !(pins & A_PIN);
  is_b_pressed = !(pins & B_PIN);
  is_x_pressed = !(pins & X_PIN);
  is_y_pressed = !(pins & Y_PIN);
  {
    std::lock_guard<std::mutex> lock(gamepad_state_mutex);
    gamepad_state.a = is_a_pressed;
    gamepad_state.b = is_b_pressed;
    gamepad_state.x = is_x_pressed;
    gamepad_state.y = is_y_pressed;
    gamepad_state.start = is_start_pressed;
    gamepad_state.select = is_select_pressed;
    gamepad_state.up = is_up_pressed;
    gamepad_state.down = is_down_pressed;
    gamepad_state.left = is_left_pressed;
    gamepad_state.right = is_right_pressed;
  }
}

static std::atomic<bool> initialized = false;
void init_input() {
  if (initialized) return;
  fmt::print("Initializing input drivers...\n");

  fmt::print("Initializing touch driver\n");
  touch_driver = std::make_shared<TouchDriver>(TouchDriver::Config{
#if TOUCH_DRIVER_USE_WRITE
      .write = std::bind(&espp::I2c::write, internal_i2c.get(), std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3),
#endif
#if TOUCH_DRIVER_USE_READ
      .read = std::bind(&espp::I2c::read, internal_i2c.get(), std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3),
#endif
#if TOUCH_DRIVER_USE_WRITE_READ
      .write_read = std::bind(&espp::I2c::write_read, internal_i2c.get(), std::placeholders::_1,
                              std::placeholders::_2, std::placeholders::_3,
                              std::placeholders::_4, std::placeholders::_5),
#endif
      .log_level = espp::Logger::Verbosity::WARN
    });

  fmt::print("Initializing touchpad\n");
  touchpad = std::make_shared<espp::TouchpadInput>(espp::TouchpadInput::Config{
      .touchpad_read = touchpad_read,
      .swap_xy = touch_swap_xy,
      .invert_x = touch_invert_x,
      .invert_y = touch_invert_y,
      .log_level = espp::Logger::Verbosity::WARN
    });

  fmt::print("initializing input driver\n");
  input_driver = std::make_shared<InputDriver>(InputDriver::Config{
      .port_0_direction_mask = PORT_0_DIRECTION_MASK,
      .port_0_interrupt_mask = PORT_0_INTERRUPT_MASK,
      .port_1_direction_mask = PORT_1_DIRECTION_MASK,
      .port_1_interrupt_mask = PORT_1_INTERRUPT_MASK,
      .write = std::bind(&espp::I2c::write, external_i2c.get(), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      .read = std::bind(&espp::I2c::read_at_register, external_i2c.get(), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .log_level = espp::Logger::Verbosity::WARN
    });

  fmt::print("Initializing keypad\n");
  keypad = std::make_shared<espp::KeypadInput>(espp::KeypadInput::Config{
      .read = keypad_read,
      .log_level = espp::Logger::Verbosity::WARN
    });

  fmt::print("Initializing input task\n");
  input_timer = std::make_shared<espp::Timer>(espp::Timer::Config{
      .name = "Input timer",
      .period = 20ms,
      .callback = []() {
        update_touchpad_input();
        update_gamepad_input();
        return false;
      },
      .log_level = espp::Logger::Verbosity::WARN});

  initialized = true;
}

extern "C" lv_indev_t *get_keypad_input_device() {
  if (!keypad) {
    fmt::print("cannot get keypad input device: keypad not initialized properly!\n");
    return nullptr;
  }
  return keypad->get_input_device();
}

extern "C" void get_input_state(struct InputState* state) {
  std::lock_guard<std::mutex> lock(gamepad_state_mutex);
  *state = gamepad_state;
}
