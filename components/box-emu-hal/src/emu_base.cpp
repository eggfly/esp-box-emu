#include "emu_base.hpp"

using namespace hal;

using DisplayDriver = espp::St7789;

// the user flag for the callbacks does two things:
// 1. Provides the GPIO level for the data/command pin, and
// 2. Sets some bits for other signaling (such as LVGL FLUSH)
static constexpr int FLUSH_BIT = (1 << (int)espp::display_drivers::Flags::FLUSH_BIT);
static constexpr int DC_LEVEL_BIT = (1 << (int)espp::display_drivers::Flags::DC_LEVEL_BIT);

// This function is called (in irq context!) just before a transmission starts.
// It will set the D/C line to the value indicated in the user field
// (DC_LEVEL_BIT).
static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    uint32_t user_flags = (uint32_t)(t->user);
    bool dc_level = user_flags & DC_LEVEL_BIT;
    gpio_set_level(lcd_dc, dc_level);
}

// This function is called (in irq context!) just after a transmission ends. It
// will indicate to lvgl that the next flush is ready to be done if the
// FLUSH_BIT is set.
static void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t)
{
    uint16_t user_flags = (uint32_t)(t->user);
    bool should_flush = user_flags & FLUSH_BIT;
    if (should_flush) {
        lv_disp_t * disp = _lv_refr_get_disp_refreshing();
        lv_disp_flush_ready(disp->driver);
    }
}

static std::shared_ptr<hal::Emu> context;

void hal::set_emu_context(std::shared_ptr<hal::Emu> emu) {
  context = emu;
}

std::shared_ptr<espp::Emu> hal::get_emu_context() {
  return context;
}

extern "C" void touchpad_read(uint8_t num_touch_points, uint16_t* x, uint16_t* y, uint8_t* btn_state) {
  if (!context) {
    return;
  }
  context->touchpad_read(num_touch_points, x, y, btn_state);
}

extern "C" void keypad_read(bool *up, bool *down, bool *left, bool *right, bool *enter, bool *escape) {
  if (!context) {
    return;
  }
  context->keypad_read(up, down, left, right, enter, escape);
}

extern "C" lv_indev_t *get_keypad_input_device() {
  if (!context) {
    return nullptr;
  }
  return context->get_keypad_input_device();
}

Hardware detect_hardware(); {
  // check box first:
  {
    // in internal scope to ensure this i2c is destructed before the next one
    // is created
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = box_hal::internal_i2c_sda,
        .scl_io_num = box_hal::internal_i2c_scl,
      });
    if (i2c.probe_device(espp::Tt22100::DEFAULT_ADDRESS)) {
      return Hardware::BOX;
    }
  }
  // check box 3:
  {
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = box_3_hal::internal_i2c_sda,
        .scl_io_num = box_3_hal::internal_i2c_scl,
      });
    if (i2c.probe_device(espp::Gt911::DEFAULT_ADDRESS)) {
      return Hardware::BOX_3;
    }
  }
  return Hardware::UNKNOWN;
}

EmuVersion detect_emu_version() {
  // check the external bus (same for all hardware) for mcp23x17 (v0), or
  // aw9523b (v1)
  espp::I2c i2c({
      .port = I2C_NUM_1,
      .sda_io_num = box_hal::external_i2c_sda,
      .scl_io_num = box_hal::external_i2c_scl,
    });
  if (i2c.probe_device(espp::Mcp23x17::DEFAULT_ADDRESS)) {
    return EmuVersion::V0;
  }
  if (i2c.probe_device(espp::Aw9523b::DEFAULT_ADDRESS)) {
    return EmuVersion::V1;
  }
  return EmuVersion::UNKNOWN;
}

void Emu::init() {

  init_sd_card();
  init_i2c();
  init_touch();
  init_keypad();
  init_input();
  init_memory();
  init_video();
  init_audio();
  init_timer();
}

void Emu::init_sd_card() {
  esp_err_t ret;

  // Options for mounting the filesystem. If format_if_mount_failed is set to
  // true, SD card will be partitioned and formatted in case when mounting
  // fails.
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };
  logger_.info("Initializing SD card");

  // Use settings defined above to initialize SD card and mount FAT filesystem.
  // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
  // Please check its source code and implement error recovery when developing
  // production applications.
  logger_.info("Using SPI peripheral");

  // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
  // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
  // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = BOX_HAL::sdcard_spi_num;
  // host.max_freq_khz = 10 * 1000;

  spi_bus_config_t bus_cfg = {
    .mosi_io_num = BOX_HAL::sdcard_mosi,
    .miso_io_num = BOX_HAL::sdcard_miso,
    .sclk_io_num = BOX_HAL::sdcard_sclk,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 8192,
  };
  spi_host_device_t host_id = (spi_host_device_t)host.slot;
  ret = spi_bus_initialize(host_id, &bus_cfg, SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK) {
    logger_.error("Failed to initialize bus.");
    return;
  }

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = BOX_HAL::sdcard_cs;
  slot_config.host_id = host_id;

  logger_.info("Mounting filesystem");
  ret = esp_vfs_fat_sdspi_mount(mount_point.c_str(), &host, &slot_config, &mount_config, &sdcard.get());

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      logger_.error("Failed to mount filesystem. "
                    "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
    } else {
      logger_.error("Failed to initialize the card ({}). "
                    "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
    }
    return;
  }
  logger_.info("Filesystem mounted");

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, sdcard.get());
}

void Emu::init_i2c() {
  gpio_num_t internal_sda = BOX_HAL::internal_i2c_sda;
  gpio_num_t internal_scl = BOX_HAL::internal_i2c_scl;
  gpio_num_t external_sda = BOX_HAL::external_i2c_sda;
  gpio_num_t external_scl = BOX_HAL::external_i2c_scl;

  logger_.info("internal_sda: {}", (int)internal_sda);
  logger_.info("internal_scl: {}", (int)internal_scl);
  logger_.info("external_sda: {}", (int)external_sda);
  logger_.info("external_scl: {}", (int)external_scl);

  // make the i2c on core 1 so that the i2c interrupts are handled on core 1
  std::atomic<bool> i2c_initialized = false;
  auto i2c_task = espp::Task::make_unique(espp::Task::Config{
      .name = "i2c",
        .callback = [this, &](auto &m, auto&cv) -> bool {
          internal_i2c = std::make_shared<espp::I2c>(espp::I2c::Config{
              .port = I2C_NUM_0,
              .sda_io_num = internal_sda,
              .scl_io_num = internal_scl,
              .sda_pullup_en = GPIO_PULLUP_ENABLE,
              .scl_pullup_en = GPIO_PULLUP_ENABLE});
          external_i2c = std::make_shared<espp::I2c>(espp::I2c::Config{
              .port = I2C_NUM_1,
              .sda_io_num = external_sda,
              .scl_io_num = external_scl,
              .sda_pullup_en = GPIO_PULLUP_ENABLE,
              .scl_pullup_en = GPIO_PULLUP_ENABLE});
          i2c_initialized = true;
          return true; // stop the task
        },
        .stack_size_bytes = 2*1024,
        .core_id = 1
        });
  i2c_task->start();
  while (!i2c_initialized) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void Emu::init_input() {
  input_driver = std::make_shared<INPUT_DRIVER>(INPUT_DRIVER::Config{
      .port_0_direction_mask = EMU_HAL::PORT_0_DIRECTION_MASK,
      .port_0_interrupt_mask = EMU_HAL::PORT_0_INTERRUPT_MASK,
      .port_1_direction_mask = EMU_HAL::PORT_1_DIRECTION_MASK,
      .port_1_interrupt_mask = EMU_HAL::PORT_1_INTERRUPT_MASK,
      .write = std::bind(&espp::I2c::write, external_i2c.get(), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      .read = std::bind(&espp::I2c::read_at_register, external_i2c.get(), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .log_level = espp::Logger::Verbosity::WARN
    });
}

void Emu::init_touch() {
  if (hardware == Hardware::BOX) {
    tt21100 = std::make_shared<espp::Tt21100>({
        .read = std::bind(&espp::I2c::read, internal_i2c.get(), std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3),
        .log_level = espp::Logger::Verbosity::WARN
      });
    touchpad = std::make_shared<espp::TouchpadInput>({
        .touchpad_read = std::bind(&espp::Tt21100::read, tt21100.get(), std::placeholders::_1),
        .swap_xy = touch_swap_xy,
        .invert_x = touch_invert_x,
        .invert_y = touch_invert_y,
        .log_level = espp::Logger::Verbosity::WARN
      });
  } else if (hardware == Hardware::BOX_3) {
    gt911 = std::make_shared<espp::Gt911>({
        .write = std::bind(&espp::I2c::write, internal_i2c.get(), std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3),
        .write_read = std::bind(&espp::I2c::write_read, internal_i2c.get(), std::placeholders::_1,
                                std::placeholders::_2, std::placeholders::_3,
                                std::placeholders::_4, std::placeholders::_5),
        .log_level = espp::Logger::Verbosity::WARN
      });
    touchpad = std::make_shared<espp::TouchpadInput>({
        .touchpad_read = std::bind(&espp::Gt911::read, gt911.get(), std::placeholders::_1),
        .swap_xy = touch_swap_xy,
        .invert_x = touch_invert_x,
        .invert_y = touch_invert_y,
        .log_level = espp::Logger::Verbosity::WARN
      });
  }
}

void Emu::init_keypad() {
  keypad = std::make_shared<espp::KeypadInput>(espp::KeypadInput::Config{
      .read = keypad_read,
      .log_level = espp::Logger::Verbosity::WARN
    });
}

void Emu::init_battery() {
  bool can_communicate = external_i2c->probe_device(BATTERY_DRIVER::DEFAULT_ADDRESS);
  if (!can_communicate) {
    logger_.error("Could not communicate with battery!");
    return;
  }
  battery_ = std::make_shared<BATTERY_DRIVER>(BATTERY_DRIVER::Config{
      .device_address = BATTERY_DRIVER::DEFAULT_ADDRESS,
      .write = std::bind(&espp::I2c::write, i2c.get(), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, i2c.get(), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    });
}

void Emu::init_timer() {
  timer = std::make_shared<espp::Timer>(espp::Timer::Config{
      .name = "I2C timer",
      .period = 30ms,
      .callback = std::bind(&Emu::timer_callback, this),
      .log_level = espp::Logger::Verbosity::WARN});
}

void Emu::touchpad_read(uint8_t* num_touch_points, uint16_t* x, uint16_t* y, uint8_t* btn_state) {
  std::lock_guard<std::mutex> lock(touchpad_data_mutex);
  *num_touch_points = touchpad_data.num_touch_points;
  *x = touchpad_data.x;
  *y = touchpad_data.y;
  *btn_state = touchpad_data.btn_state;
}

void Emu::keypad_read(bool *up, bool *down, bool *left, bool *right, bool *enter, bool *escape) {
  std::lock_guard<std::mutex> lock(input_state_mutex);
  *up = input_state.up;
  *down = input_state.down;
  *left = input_state.left;
  *right = input_state.right;
  *enter = input_state.enter;
  *escape = input_state.escape;
}

lv_indev_t *Emu::get_keypad_input_device() {
  if (!keypad) {
    logger_.error("cannot get keypad input device: keypad not initialized properly!");
    return nullptr;
  }
  return keypad->get_input_device();
}

void Emu::update_gamepad_input() {
  if (!input_driver) return;
  std::error_code ec;
  auto pins = input_driver->get_pins(ec);
  if (ec) {
    logger_.error("error getting pins: {}", ec.message());
    return;
  }
  pins = pins ^ EMU_HAL::INVERT_MASK;
  {
    std::lock_guard<std::mutex> lock(gamepad_state_mutex);
    gamepad_state.a = (bool)(pins & EMU_HAL::A_PIN);
    gamepad_state.b = (bool)(pins & EMU_HAL::B_PIN);
    gamepad_state.x = (bool)(pins & EMU_HAL::X_PIN);
    gamepad_state.y = (bool)(pins & EMU_HAL::Y_PIN);
    gamepad_state.start = (bool)(pins & EMU_HAL::START_PIN);
    gamepad_state.select = (bool)(pins & EMU_HAL::SELECT_PIN);
    gamepad_state.up = (bool)(pins & EMU_HAL::UP_PIN);
    gamepad_state.down = (bool)(pins & EMU_HAL::DOWN_PIN);
    gamepad_state.left = (bool)(pins & EMU_HAL::LEFT_PIN);
    gamepad_state.right = (bool)(pins & EMU_HAL::RIGHT_PIN);
  }
  // check the volume pins and send out events if they're pressed / released
  bool volume_up = (bool)(pins & EMU_HAL::VOL_UP_PIN);
  bool volume_down = (bool)(pins & EMU_HAL::VOL_DOWN_PIN);
  int volume_change = volume_up * 10 + volume_down * -10;
  if (volume_change) {
    logger_.info("volume change: {}", volume_change);
    // change the volume
    set_audio_volume(std::clamp<int>(get_audio_volume() + volume_change, 0, 100));
    // send out a volume changed event
    espp::EventManager::get().publish(volume_changed_topic, {});
  }
  // TODO: check the battery alert pin and if it's low, send out a battery alert event
}

void Emu::update_battery_info() {
  std::error_code ec;
  // get the voltage (V)
  auto voltage = battery_driver->get_battery_voltage(ec);
  if (ec) return;
  // get the state of charge (%)
  auto soc = battery_driver->get_battery_percentage(ec);
  if (ec) return;
  // get the charge rate (+/- % per hour)
  auto charge_rate = battery_driver->get_battery_charge_rate(ec);
  if (ec) return;

  auto new_battery_info = BatteryInfo{
    .voltage = voltage,
    .level = soc,
    .charge_rate = charge_rate,
  };
  {
    std::lock_guard<std::mutex> lock(battery_info_mutex);
    battery_info = new_battery_info;
  }

  // now publish a BatteryInfo struct to the battery_topic
  std::vector<uint8_t> battery_info_data;
  // fmt::print("Publishing battery info: {}\n", battery_info);
  auto bytes_serialized = espp::serialize(new_battery_info, battery_info_data);
  if (bytes_serialized == 0) {
    return;
  }
  espp::EventManager::get().publish(battery_topic, battery_info_data);
}

void Emu::update_touchpad_input() {
  // get the latest data from the device
  std::error_code ec;
  bool new_data = touch_driver->update(ec);
  if (ec) {
    logger_.error("error updating touch_driver: {}", ec.message());
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

bool Emu::timer_callback() {
  update_touchpad_input();
  update_gamepad_input();
  return false;
}

void Emu::init_memory() {
  // allocate memory for the ROM and make sure it's on the SPIRAM
  romdata = (uint8_t*)heap_caps_malloc(4*1024*1024, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  if (romdata == nullptr) {
    logger_.error("Couldn't allocate memory for ROM!");
  }
}

uint16_t Emu::make_color(uint8_t r, uint8_t g, uint8_t b) {
  return lv_color_make(r,g,b).full;
}

void Emu::init_video() {
    esp_err_t ret;

    logger_.info("initializing spi lcd");
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = BOX_HAL::lcd_mosi;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = BOX_HAL::lcd_sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = frame_buffer_size * sizeof(lv_color_t) + 10;

    memset(&lcd_devcfg, 0, sizeof(lcd_devcfg));
    lcd_devcfg.mode = 0;
    // lcd_devcfg.flags = SPI_DEVICE_NO_RETURN_RESULT;
    lcd_devcfg.clock_speed_hz = BOX_HAL::lcd_clock_speed;
    lcd_devcfg.input_delay_ns = 0;
    lcd_devcfg.spics_io_num = BOX_HAL::lcd_cs;
    lcd_devcfg.queue_size = BOX_HAL::spi_queue_size;
    lcd_devcfg.pre_cb = lcd_spi_pre_transfer_callback;
    lcd_devcfg.post_cb = lcd_spi_post_transfer_callback;

    //Initialize the SPI bus
    ret = spi_bus_initialize(BOX_HAL::lcd_spi_num, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(BOX_HAL::lcd_spi_num, &lcd_devcfg, &lcd_spi);
    ESP_ERROR_CHECK(ret);
    // initialize the controller
    DisplayDriver::initialize(espp::display_drivers::Config{
            .lcd_write = lcd_write,
            .lcd_send_lines = lcd_send_lines,
            .reset_pin = BOX_HAL::lcd_reset,
            .data_command_pin = BOX_HAL::lcd_dc,
            .reset_value = BOX_HAL::reset_value,
            .invert_colors = BOX_HAL::invert_colors,
            .mirror_x = BOX_HAL::mirror_x,
            .mirror_y = BOX_HAL::mirror_y
        });
    // initialize the display / lvgl
    using namespace std::chrono_literals;
    display = std::make_shared<espp::Display>(espp::Display::AllocatingConfig{
            .width = display_width,
            .height = display_height,
            .pixel_buffer_size = pixel_buffer_size,
            .flush_callback = DisplayDriver::flush,
            .backlight_pin = BOX_HAL::backlight,
            .backlight_on_value = BOX_HAL::backlight_value,
            .update_period = 5ms,
            .double_buffered = true,
            .allocation_flags = MALLOC_CAP_8BIT | MALLOC_CAP_DMA,
            .rotation = rotation,
            .software_rotation_enabled = true,
        });

    framebuffer0 = (uint8_t*)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    framebuffer1 = (uint8_t*)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);

    logger_.info("initializing video task");
    video_queue = xQueueCreate(1, sizeof(uint16_t*));
    video_task = std::make_unique<espp::Task>(espp::Task::Config{
        .name = "video task",
        .callback = video_task,
        .stack_size_bytes = 6*1024,
        .priority = 20,
        .core_id = 1
      });
    video_task->start();
}

void Emu::push_frame(const void* frame) {
  xQueueSend(video_queue, &frame, 10 / portTICK_PERIOD_MS);
}

bool Emu::video_task_callback(std::mutex& m, std::condition_variable& cv) {
  const void *_frame_ptr;
  if (xQueuePeek(video_queue, &_frame_ptr, 100 / portTICK_PERIOD_MS) != pdTRUE) {
    // we couldn't get anything from the queue, return
    return false;
  }
  if (_frame_ptr == nullptr) {
    // make sure we clear the queue
    xQueueReceive(video_queue, &_frame_ptr, 10 / portTICK_PERIOD_MS);
    // we got a nullptr, return
    return false;
  }
  static constexpr int num_lines_to_write = NUM_ROWS_IN_FRAME_BUFFER;
  static int vram_index = 0; // has to be static so that it persists between calls
  const int x_offset = get_x_offset();
  const int y_offset = get_y_offset();
  const uint16_t* _palette = get_palette();
  if (is_native()) {
    for (int y=0; y<emulator_display_height; y+= num_lines_to_write) {
      uint16_t* _buf = vram_index ? (uint16_t*)get_vram1() : (uint16_t*)get_vram0();
      vram_index = vram_index ? 0 : 1;
      int num_lines = std::min<int>(num_lines_to_write, emulator_display_height-y);
      if (has_palette()) {
        const uint8_t* _frame = (const uint8_t*)_frame_ptr;
        for (int i=0; i<num_lines; i++) {
          for (int j=0; j<emulator_display_width; j++) {
            int index = (y+i)*emulator_native_pitch + j;
            _buf[i*emulator_display_width + j] = _palette[_frame[index] % palette_size];
          }
        }
      } else {
        const uint16_t* _frame = (const uint16_t*)_frame_ptr;
        for (int i=0; i<num_lines; i++)
          for (int j=0; j<emulator_display_width; j++)
            _buf[i*emulator_display_width + j] = _frame[(y+i)*native_pitch + j];
      }
      lcd_write_frame(x_offset, y + y_offset, emulator_display_width, num_lines, (uint8_t*)&_buf[0]);
    }
  } else {
    // we are scaling the screen (and possibly using a custom palette)
    // if we don't have a custom palette, we just need to scale/fill the frame
    float y_scale = (float)emulator_display_height/emulator_native_height;
    float x_scale = (float)emulator_display_width/emulator_native_width;
    int max_y = display_height;
    int max_x = std::clamp<int>(x_scale * emulator_native_width, 0, display_width);
    for (int y=0; y<max_y; y+=num_lines_to_write) {
      // each iteration of the loop, we swap the vram index so that we can
      // write to the other buffer while the other one is being transmitted
      int i = 0;
      uint16_t* _buf = vram_index ? (uint16_t*)get_vram1() : (uint16_t*)get_vram0();
      vram_index = vram_index ? 0 : 1;
      for (; i<num_lines_to_write; i++) {
        int _y = y+i;
        if (_y >= max_y) {
          break;
        }
        int source_y = (float)_y/y_scale;
        // shoudl i put this around the outer loop or is this loop a good
        // balance for perfomance of the check?
        if (has_palette()) {
          const uint8_t* _frame = (const uint8_t*)_frame_ptr;
          for (int x=0; x<max_x; x++) {
            int source_x = (float)x/x_scale;
            int index = source_y*emulator_native_pitch + source_x;
            _buf[i*max_x + x] = _palette[_frame[index] % palette_size];
          }
        } else {
          const uint16_t* _frame = (const uint16_t*)_frame_ptr;
          for (int x=0; x<max_x; x++) {
            int source_x = (float)x/x_scale;
            _buf[i*max_x + x] = _frame[source_y*emulator_native_pitch + source_x];
          }
        }
      }
      lcd_write_frame(0 + x_offset, y, max_x, i, (uint8_t*)&_buf[0]);
    }
  }

  // we don't have to worry here since we know there was an item in the queue
  // since we peeked earlier.
  xQueueReceive(video_queue, &_frame_ptr, 10 / portTICK_PERIOD_MS);
  return false;
}


///////////////////////////
// Audio functions
///////////////////////////

void Emu::init_audio() {
  set_es8311_write(std::bind(&espp::I2c::write, internal_i2c.get(),
                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  set_es8311_read(std::bind(&espp::I2c::read_at_register, internal_i2c.get(),
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));


  logger_.info("Configuring power control I/O (for audio output)");
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = 1ULL << (int)BOX_HAL::sound_power_pin;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);
  gpio_set_level(BOX_HAL::sound_power_pin, 1);

  logger_.info("initializing i2s driver");
  auto ret_val = ESP_OK;
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BOX_HAL::i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_handle));
  i2s_std_clk_config_t clock_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE);
  i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  i2s_std_config_t std_cfg = {
    .clk_cfg = clock_cfg,
    .slot_cfg = slot_cfg,
    .gpio_cfg = {
      .mclk = BOX_HAL::i2s_mck_io,
      .bclk = BOX_HAL::i2s_bck_io,
      .ws = BOX_HAL::i2s_ws_io,
      .dout = BOX_HAL::i2s_do_io,
      .din = BOX_HAL::i2s_di_io,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_handle));

  logger_.info("initializing es8311 codec");
  audio_hal_codec_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
  cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_LINE1;
  cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  cfg.i2s_iface.samples = AUDIO_HAL_16K_SAMPLES;

  auto ret_val |= es8311_codec_init(&cfg);
  ret_val |= es8311_set_bits_per_sample(cfg.i2s_iface.bits);
  ret_val |= es8311_config_fmt((es_i2s_fmt_t)cfg.i2s_iface.fmt);
  ret_val |= es8311_codec_set_voice_volume(audio_volume);
  ret_val |= es8311_codec_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);

  audio_buffer = (int16_t*)heap_caps_malloc(sizeof(int16_t) * AUDIO_BUFFER_SIZE + 10, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
}

void Emu::update_volume_output() {
 if (audio_muted) {
   es8311_codec_set_voice_volume(0);
 } else {
   es8311_codec_set_voice_volume(audio_volume);
 }
}

void Emu::set_audio_volume(int new_volume) {
 audio_volume = std::clamp<int>(new_volume, 0, 100);
 update_volume_output();
}

void Emu::set_audio_muted(bool new_audio_muted) {
  audio_muted = new_audio_muted;
  update_volume_output();
}

void Emu::play_audio(const uint8_t *data, size_t num_bytes) {
  size_t bytes_written = 0;
  auto err = i2s_channel_write(i2s_tx_handle, data, num_bytes, &bytes_written, 100);
  if(num_bytes != bytes_written) {
    logger_.error("ERROR to write {} != written {}", num_bytes, bytes_written);
  }
  if (err != ESP_OK) {
    logger_.error("ERROR writing i2s channel: {}, '{}'", err, esp_err_to_name(err));
  }
}

///////////////////////////
// ROM Loading Functions
///////////////////////////

size_t Emu::load_rom(const std::string& filename) {
  romsize = 0;
  if (!romdata) {
    logger_.error("ROM data not allocated!");
    return 0;
  }
  // load the ROM into memory
  std::ifstream romfile(filename, std::ios::binary);
  if (!romfile) {
    logger_.error("Couldn't open ROM file: {}", filename);
    return 0;
  }
  romfile.seekg(0, std::ios::end);
  romsize = romfile.tellg();
  romfile.seekg(0, std::ios::beg);
  romfile.read((char*)romdata, romsize);
  romfile.close();
  return romsize;
}

///////////////////////////
// LCD SPI Functions
///////////////////////////

void Emu::lcd_wait_lines() {
    spi_transaction_t *rtrans;
    esp_err_t ret;
    // fmt::print("Waiting for {} queued transactions\n", num_queued_trans);
    // Wait for all transactions to be done and get back the results.
    while (num_queued_trans) {
        ret = spi_device_get_trans_result(lcd_spi, &rtrans, 10 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) {
            logger_.error("Could not get trans result: {} '{}'", ret, esp_err_to_name(ret));
        }
        num_queued_trans--;
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

void Emu::lcd_write(const uint8_t *data, size_t length, uint32_t user_data) {
    if (length == 0) {
        return;
    }
    lcd_wait_lines();
    esp_err_t ret;
    memset(&lcd_trans[0], 0, sizeof(spi_transaction_t));
    lcd_trans[0].length = length * 8;
    lcd_trans[0].user = (void*)user_data;
    // look at the length of the data and use tx_data if it is <= 32 bits
    if (length <= 4) {
        // copy the data pointer to lcd_trans[0].tx_data
        memcpy(lcd_trans[0].tx_data, data, length);
        lcd_trans[0].flags = SPI_TRANS_USE_TXDATA;
    } else {
        lcd_trans[0].tx_buffer = data;
        lcd_trans[0].flags = 0;
    }
    ret = spi_device_queue_trans(lcd_spi, &lcd_trans[0], 10 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        logger_.error("Couldn't queue trans: {} '{}'", ret, esp_err_to_name(ret));
    } else {
        num_queued_trans++;
    }
}

void Emu::lcd_send_lines(int xs, int ys, int xe, int ye, const uint8_t *data, uint32_t user_data) {
    // if we haven't waited by now, wait here...
    lcd_wait_lines();
    esp_err_t ret;
    size_t length = (xe-xs+1)*(ye-ys+1)*2;
    if (length == 0) {
        logger_.error("Bad length: ({},{}) to ({},{})", xs, ys, xe, ye);
    }
    // initialize the spi transactions
    for (int i=0; i<6; i++) {
        memset(&lcd_trans[i], 0, sizeof(spi_transaction_t));
        if ((i&1)==0) {
            //Even transfers are commands
            lcd_trans[i].length = 8;
            lcd_trans[i].user = (void*)0;
        } else {
            //Odd transfers are data
            lcd_trans[i].length = 8*4;
            lcd_trans[i].user = (void*)DC_LEVEL_BIT;
        }
        lcd_trans[i].flags = SPI_TRANS_USE_TXDATA;
    }
    lcd_trans[0].tx_data[0] = (uint8_t)DisplayDriver::Command::caset;
    lcd_trans[1].tx_data[0] = (xs)>> 8;
    lcd_trans[1].tx_data[1] = (xs)&0xff;
    lcd_trans[1].tx_data[2] = (xe)>>8;
    lcd_trans[1].tx_data[3] = (xe)&0xff;
    lcd_trans[2].tx_data[0] = (uint8_t)DisplayDriver::Command::raset;
    lcd_trans[3].tx_data[0] = (ys)>>8;
    lcd_trans[3].tx_data[1] = (ys)&0xff;
    lcd_trans[3].tx_data[2] = (ye)>>8;
    lcd_trans[3].tx_data[3] = (ye)&0xff;
    lcd_trans[4].tx_data[0] = (uint8_t)DisplayDriver::Command::ramwr;
    lcd_trans[5].tx_buffer = data;
    lcd_trans[5].length = length*8;
    // undo SPI_TRANS_USE_TXDATA flag
    lcd_trans[5].flags = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL;
    // we need to keep the dc bit set, but also add our flags
    lcd_trans[5].user = (void*)(DC_LEVEL_BIT | user_data);
    //Queue all transactions.
    for (int i=0; i<6; i++) {
        ret = spi_device_queue_trans(lcd_spi, &lcd_trans[i], 10 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) {
            fmt::print("Couldn't queue trans: {} '{}'\n", ret, esp_err_to_name(ret));
        } else {
            num_queued_trans++;
        }
    }
    //When we are here, the SPI driver is busy (in the background) getting the
    //transactions sent. That happens mostly using DMA, so the CPU doesn't have
    //much to do here. We're not going to wait for the transaction to finish
    //because we may as well spend the time calculating the next line. When that
    //is done, we can call lcd_wait_lines, which will wait for the transfers
    //to be done and check their status.
}
