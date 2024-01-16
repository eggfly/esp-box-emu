#pragma once

#include <sdkconfig.h>

#include <memory>

#include "event_manager.hpp"
#include "display.hpp"
#include "max1704x.hpp"
#include "oneshot_adc.hpp"
#include "serialization.hpp"
#include "st7789.hpp"

#if CONFIG_HARDWARE_BOX
#include "box.hpp"
#elif CONFIG_HARDWARE_BOX_3
#include "box_3.hpp"
#else
#error "Invalid module selection"
#endif

// these can coexist
#include "emu_v0.hpp"
#include "emu_v1.hpp"

#include "i2s_audio.h"
#include "input.h"
#include "spi_lcd.h"

#include "audio_task.hpp"
#include "battery.hpp"
#include "fs_init.hpp"
#include "hal_events.hpp"
#include "hal_i2c.hpp"
#include "mmap.hpp"
#include "statistics.hpp"
#include "usb.hpp"
#include "video_setting.hpp"
#include "video_task.hpp"

namespace hal {
  void init();
  std::shared_ptr<espp::Display> get_display();
}
