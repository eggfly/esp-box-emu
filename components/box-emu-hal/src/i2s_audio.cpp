#include "i2s_audio.h"

#include <atomic>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_check.h"

#include "task.hpp"
#include "event_manager.hpp"

#include "hal_events.hpp"

#include "es7210.hpp"
#include "es8311.hpp"

#include "hal.hpp"
#include "hal_i2c.hpp"

// es7210 is for audio input codec
static esp_err_t es7210_init_default(void)
{
  fmt::print("initializing es7210 codec...\n");
  esp_err_t ret_val = ESP_OK;
  audio_hal_codec_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE;
  cfg.adc_input = AUDIO_HAL_ADC_INPUT_ALL;
  cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  cfg.i2s_iface.samples = AUDIO_HAL_16K_SAMPLES;
  ret_val |= es7210_adc_init(&cfg);
  ret_val |= es7210_adc_config_i2s(cfg.codec_mode, &cfg.i2s_iface);
  ret_val |= es7210_adc_set_gain((es7210_input_mics_t)(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2), GAIN_37_5DB);
  ret_val |= es7210_adc_set_gain((es7210_input_mics_t)(ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4), GAIN_0DB);
  ret_val |= es7210_adc_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);

  if (ESP_OK != ret_val) {
    fmt::print("Failed initialize codec\n");
  }

  return ret_val;
}


static std::unique_ptr<espp::Task> mute_task;
static QueueHandle_t gpio_evt_queue;

static void gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void init_mute_button(void) {
  // create the gpio event queue
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  // setup gpio interrupts for mute button
  gpio_config_t io_conf;
  memset(&io_conf, 0, sizeof(io_conf));
  // interrupt on any edge (since MUTE is connected to flipflop, see note below)
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = (1<<(int)mute_pin);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);

  // update the mute state (since it's a flip-flop and may have been set if we
  // restarted without power loss)
  set_muted(!gpio_get_level(mute_pin));

  // create a task on core 1 for initializing the gpio interrupt so that the
  // gpio ISR runs on core 1
  auto gpio_task = espp::Task::make_unique(espp::Task::Config{
      .name = "gpio",
        .callback = [](auto &m, auto&cv) -> bool {
          gpio_install_isr_service(0);
          gpio_isr_handler_add(mute_pin, gpio_isr_handler, (void*) mute_pin);
          return true; // stop the task
        },
      .stack_size_bytes = 2*1024,
      .core_id = 1
    });
  gpio_task->start();

  // register that we publish the mute button state
  espp::EventManager::get().add_publisher(mute_button_topic, "i2s_audio");

  // start the gpio task
  mute_task = espp::Task::make_unique(espp::Task::Config{
      .name = "mute",
      .callback = [](auto &m, auto&cv) -> bool {
        static gpio_num_t io_num;
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
          // see if it's the mute button
          if (io_num == mute_pin) {
            // invert the state since these are active low switches
            bool pressed = !gpio_get_level(io_num);
            // NOTE: the MUTE is actually connected to a flip-flop which holds
            // state, so pressing it actually toggles the state that we see on
            // the ESP pin. Therefore, when we get an edge trigger, we should
            // read the state to know whether to be muted or not.
            set_muted(pressed);
            // simply publish that the mute button was presssed
            espp::EventManager::get().publish(mute_button_topic, {});
          }
        }
        // don't want to stop the task
        return false;
      },
      .stack_size_bytes = 4*1024,
    });
  mute_task->start();
}

static bool initialized = false;
void audio_init() {
  if (initialized) return;

  /* Config power control IO */

  i2s_driver_init();
  es7210_init_default();
  es8311_init_default();

  audio_buffer0 = (int16_t*)heap_caps_malloc(sizeof(int16_t) * AUDIO_BUFFER_SIZE + 10, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  // audio_buffer1 = (int16_t*)heap_caps_malloc(sizeof(int16_t) * AUDIO_BUFFER_SIZE + 10, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);

  // now initialize the mute gpio
  init_mute_button();

  initialized = true;
}

void audio_deinit() {
  if (!initialized) return;
  i2s_channel_disable(tx_handle);
  i2s_channel_disable(rx_handle);
  i2s_del_channel(tx_handle);
  i2s_del_channel(rx_handle);
  initialized = false;
}

void audio_play_frame(const uint8_t *data, uint32_t num_bytes) {
  size_t bytes_written = 0;
  auto err = ESP_OK;
  err = i2s_channel_write(tx_handle, data, num_bytes, &bytes_written, 1000);
  if(num_bytes != bytes_written) {
    fmt::print("ERROR to write {} != written {}\n", num_bytes, bytes_written);
  }
  if (err != ESP_OK) {
    fmt::print("ERROR writing i2s channel: {}, '{}'\n", err, esp_err_to_name(err));
  }
}
