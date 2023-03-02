#include "nes.hpp"

#include <string>

#include "NES.h"

#include "fs_init.hpp"
#include "format.hpp"
#include "spi_lcd.h"
#include "st7789.hpp"

static std::atomic<bool> scaled = false;
static std::atomic<bool> filled = true;

NES* nes;

#define GET_R_(c) ((c >> 16)&0xFF)
#define GET_G_(c) ((c >> 8)&0xFF)
#define GET_B_(c) ((c >> 0)&0xFF)
constexpr uint32_t old_palette[] = { 0xff666666, 0xff882a00, 0xffa71214, 0xffa4003b, 0xff7e005c, 0xff40006e, 0xff00066c, 0xff001d56, 0xff003533, 0xff00480b, 0xff005200, 0xff084f00, 0xff4d4000, 0xff000000, 0xff000000, 0xff000000, 0xffadadad, 0xffd95f15, 0xffff4042, 0xfffe2775, 0xffcc1aa0, 0xff7b1eb7, 0xff2031b5, 0xff004e99, 0xff006d6b, 0xff008738, 0xff00930c, 0xff328f00, 0xff8d7c00, 0xff000000, 0xff000000, 0xff000000, 0xfffffeff, 0xffffb064, 0xffff9092, 0xffff76c6, 0xffff6af3, 0xffcc6efe, 0xff7081fe, 0xff229eea, 0xff00bebc, 0xff00d888, 0xff30e45c, 0xff82e045, 0xffdecd48, 0xff4f4f4f, 0xff000000, 0xff000000, 0xfffffeff, 0xffffdfc0, 0xffffd2d3, 0xffffc8e8, 0xffffc2fb, 0xffeac4fe, 0xffc5ccfe, 0xffa5d8f7, 0xff94e5e4, 0xff96efcf, 0xffabf4bd, 0xffccf3b3, 0xfff2ebb5, 0xffb8b8b8, 0xff000000, 0xff000000 };
uint16_t *new_palette;

void set_nes_video_original() {
  scaled = false;
  filled = false;
}

void set_nes_video_fit() {
  scaled = true;
  filled = false;
}

void set_nes_video_fill() {
  scaled = false;
  filled = true;
}

void init_nes(const std::string& rom_filename, uint8_t *romdata, size_t rom_data_size) {
  static bool initialized = false;
  if (!initialized) {
    int num_p = sizeof(old_palette) / sizeof(uint32_t);
    new_palette = new uint16_t[num_p];
    for (int i=0; i<num_p; i++) {
      uint32_t c = old_palette[i];
      new_palette[i] = make_color(GET_R_(c), GET_G_(c), GET_B_(c));
    }
    setPalette(new_palette);
    // TODO: update sram
    // TODO: update how rom file is loaded
    std::string SRAM_path = rom_filename + ".srm";
    nes = new NES(rom_filename.c_str(), SRAM_path.c_str(), get_vram0(), get_vram1());
    // setup audio (nes->apu->stream)
    // setup video (nes->ppu->front)
  }
  initialized = true;
}

void run_nes_rom() {
  static int num_frames = 0;
  static float elapsed = 0.0f;
  auto start = std::chrono::high_resolution_clock::now();
  // TODO: input
  nes->controller1->buttons = 0; // set these bitmasks appropriately
  nes->controller2->buttons = 0; // set the se bitmasks appropriately
  // emulate
  emulate(nes, 0.1f/60.0f);
  // frame rate should be 60 FPS, so 1/60th second is what we want to sleep for
  auto end = std::chrono::high_resolution_clock::now();
  elapsed += std::chrono::duration<float>(end-start).count();
  if (num_frames++ == 60) {
    fmt::print("fps: {:.2f}\n", num_frames / elapsed);
    num_frames = 0;
    elapsed = 0;
  }
  auto delay = std::chrono::duration<float>(1.0f/60.0f);
  std::this_thread::sleep_until(start + delay);
}

void load_nes(std::string_view save_path) {
}

void save_nes(std::string_view save_path) {
}

void deinit_nes() {
}
