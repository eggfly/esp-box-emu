#pragma once

#include <atomic>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>

#include <sys/stat.h>
#include <errno.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_partition.h>
#include <esp_vfs_fat.h>
#include <hal/spi_types.h>
#include <nvs_flash.h>
#include <sdmmc_cmd.h>
#include <spi_flash_mmap.h>

#include "aw9523.hpp"
#include "event_manager.hpp"
#include "i2c.hpp"
#include "logger.hpp"
#include "max1704x.hpp"
#include "mcp23x17.hpp"
#include "timer.hpp"

#include "es8311.hpp"
#include "battery_info.hpp"
#include "input_state.hpp"
#include "touchpad_data.hpp"

#include "lvgl.h"

// These functions need to be implemented for lvgl keypad and touchpad drivers
extern "C" lv_indev_t *get_keypad_input_device();
extern "C" void touchpad_read(unsigned char *num_touch_points, unsigned short* x, unsigned short* y, unsigned char* btn_state);

namespace hal {
  // Types of hardware we can run on (dev-kits)
  enum class Hardware {
    UNKNOWN,
    BOX,
    BOX_3,
  };

  // Version of our emulator / gamepad hardware
  enum class EmuVersion {
    UNKNOWN,
    V0,
    V1,
  };

  // Event topics
  [[maybe_unused]]
  static const std::string mute_button_topic = "mute";
  [[maybe_unused]]
  static const std::string volume_changed_topic = "volume";
  [[maybe_unused]]
  static const std::string video_setting_topic = "video";
  [[maybe_unused]]
  static const std::string battery_topic = "battery";

  // uSD card mount point
  [[maybe_unused]]
  static const std::string mount_point = "/sdcard";

  // forward declaration
  class Emu;

  // audio constants
  static constexpr int AUDIO_SAMPLE_RATE = 32000;
  static constexpr int AUDIO_BUFFER_SIZE = AUDIO_SAMPLE_RATE / 5;
  static constexpr int AUDIO_SAMPLE_COUNT = AUDIO_SAMPLE_RATE / 60;

  // video constants
  static constexpr int NUM_ROWS_IN_FRAME_BUFFER = 50;
  static constexprt size_t display_width = 320;
  static constexprt size_t display_height = 240;
  static constexpr size_t pixel_buffer_size = display_width * NUM_ROWS_IN_FRAME_BUFFER;
  static constexpr size_t frame_buffer_size = (((display_width) * sizeof(uint16_t)) * display_height);
  [[maybe_unused]]
  uint16_t make_color(uint8_t r, uint8_t g, uint8_t b);

  // For use with c-style callbacks (such as needed by lvgl)
  [[maybe_unused]]
  static void set_emu_context(std::shared_ptr<Emu> emu);
  [[maybe_unused]]
  static std::shared_ptr<Emu> get_emu_context();

  // For determining what kind of subclass to instantiate
  [[maybe_unused]]
  static Hardware detect_hardware();
  [[maybe_unused]]
  static EmuVersion detect_emu_version();

  // Class storing all the configuration options for the emulator and the
  // hardware it's running on. Can be subclassed for specific hardware or used
  // as a template class.
  template <typename BOX_HAL, typename EMU_HAL, typename INPUT_DRIVER, typename BATTERY_DRIVER=espp::Max1704x>
  class Emu {
  public:
    virtual Emu();
    virtual ~Emu();

    std::shared_ptr<espp::I2c> get_internal_i2c() { return internal_i2c; }
    std::shared_ptr<espp::I2c> get_external_i2c() { return external_i2c; }

    size_t load_rom(const std::string& filename);
    uint8_t get_romdata() { return romdata; }
    size_t get_romsize() const { return romsize; }

    // Implementations of lvgl input device callbacks
    void touchpad_read(uint8_t* num_touch_points, uint16_t* x, uint16_t* y, uint8_t* btn_state);
    void keypad_read(bool *up, bool *down, bool *left, bool *right, bool *enter, bool *escape);
    lv_indev_t *get_keypad_input_device();

    // Set the volume to a value between 0 and 100
    virtual void set_audio_volume(int new_audio_volume);
    int get_audio_volume() const { return audio_volume; }

    // Mute or unmute the audio (NOTE: the physical mute button can still override this)
    virtual void set_audio_muted(bool new_audio_muted);
    bool get_audio_muted() const { return audio_muted; }

    int16_t *get_audio_buffer() { return audio_buffer; }
    void play_audio(const uint8_t *data, size_t size);

    // video
    void push_frame(const void* frame);

    void set_emulator_display_size(size_t width, size_t height) {
      emulator_display_width = width;
      emulator_display_height = height;
    }
    void set_emulator_native_size(size_t width, size_t height, int pitch = -1) {
      emulator_native_width = width;
      emulator_native_height = height;
      emulator_native_pitch = (pitch == -1) ? width : pitch;
    }
    void set_palette(const uint16_t *_palette, size_t size = 256) {
      palette = _palette;
      palette_size = size;
    }

    uint16_t *get_vram0() { return display->vram0(); }
    uint16_t *get_vram1() { return display->vram1(); }
    uint8_t *get_framebuffer0() { return framebuffer0; }
    uint8_t *get_framebuffer1() { return framebuffer1; }

    void set_display_brightness(int new_brightness) { display->set_brightness(new_brightness); }
    int get_display_brightness() const { return display->get_brightness(); }

  protected:
    static constexpr auto MCLK_MULTIPLE = I2S_MCLK_MULTIPLE_256;
    static constexpr int MCLK_FREQ_HZ = AUDIO_SAMPLE_RATE * MCLK_MULTIPLE;

    virtual bool timer_callback();
    virtual void update_gamepad_input();
    virtual void update_touchpad_input();
    virtual void update_battery_info();
    virtual void update_volume_output();

    virtual bool video_task_callback(std::mutex& m, std::condition_variable& cv);

    void lcd_wait_lines();
    void lcd_write(const uint8_t *data, size_t length, uint32_t user_data);
    void lcd_write(const uint8_t *data, size_t length, uint32_t user_data);
    void lcd_send_lines(int xs, int ys, int xe, int ye, const uint8_t *data, uint32_t user_data);

    bool has_palette() { return palette != nullptr; }
    bool is_native() {
      return emulator_native_width == emulator_display_width &&
        emulator_native_height == emulator_display_height;
    }
    int get_x_offset() { return (display_width - emulator_display_width) / 2; }
    int get_y_offset() { return (display_height - emulator_display_height) / 2; }
    const uint16_t* get_palette() { return palette; }

    virtual void init();
    virtual void init_sd_card();
    virtual void init_i2c();
    virtual void init_touch();
    virtual void init_keypad();
    virtual void init_input();
    virtual void init_battery();
    virtual void init_timer();
    virtual void init_memory();
    virtual void init_video();
    virtual void init_audio();

    uint8_t *romdata{nullptr};
    size_t romsize{0};

    InputState input_state{};
    std::mutex input_state_mutex;
    TouchpadData touchpad_data{};
    std::mutex touchpad_data_mutex;
    BatteryInfo battery_info{};
    std::mutex battery_info_mutex;

    // for audio
    int audio_volume{50};
    bool audio_muted{false};
    i2s_chan_handle_t i2s_tx_handle{nullptr};
    int16_t *audio_buffer{nullptr};

    // for video
    size_t emulator_display_width{0};
    size_t emulator_display_height{0};
    size_t emulator_native_width{0};
    size_t emulator_native_height{0};
    int emulator_native_pitch{-1};
    const uint16_t* palette{nullptr};
    size_t palette_size{256}; // number of elements in the palette
    uint8_t *framebuffer0{nullptr};
    uint8_t *framebuffer1{nullptr};
    spi_device_handle_t lcd_spi;
    spi_device_interface_config_t lcd_devcfg;
    std::shared_ptr<espp::Display> display{nullptr};
    std::unique_ptr<espp::Task> video_task;
    QueueHandle_t video_queue;

    // Transaction descriptors. Declared static so they're not allocated on the
    // stack; we need this memory even when this function is finished because the
    // SPI driver needs access to it even while we're already calculating the next
    // line.
    static constexpr int spi_queue_size = 6;
    spi_transaction_t lcd_trans[spi_queue_size];
    std::atomic<int> num_queued_trans = 0;

    // for battery
    std::shared_ptr<BATTERY_DRIVER> battery_driver{nullptr};

    // for storage
    std::shared_ptr<sdmmc_card_t> sd_card{nullptr};

    // for input
    std::shared_ptr<INPUT_DRIVER> input_driver{nullptr};
    std::shared_ptr<espp::Tt21100> tt21100{nullptr};
    std::shared_ptr<espp::Gt911> gt911{nullptr};
    std::shared_ptr<espp::TouchpadInput> touchpad{nullptr};
    std::shared_ptr<espp::KeypadInput> keypad{nullptr};
    std::unique_ptr<espp::Timer> timer{nullptr};

    std::shared_ptr<espp::I2c> internal_i2c{nullptr};
    std::shared_ptr<espp::I2c> external_i2c{nullptr};
  };
}
