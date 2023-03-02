#include "nes.hpp"

#include <string>

#include "nes/cartridge/cartridge.h"
#include "nes/cartridge/rom_file.h"
#include "nes/cartridge/parse_rom.h"
#include "nes/joy/controllers/standard.h"
#include "nes/nes.h"
#include "nes/params.h"

#include "fs_init.hpp"
#include "format.hpp"
#include "input.h"
#include "spi_lcd.h"
#include "st7789.hpp"

static NES_Params nes_params;
static JOY_Standard controller { "P1" };
static struct InputState state;
static std::shared_ptr<NES> nes;
Cartridge *cart;
static bool initialized = false;
static std::atomic<bool> scaled = false;
static std::atomic<bool> filled = true;

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
  if (!initialized) {
    nes_params.apu_sample_rate = 32000;
    nes_params.speed = 100;
    nes = std::make_shared<NES>(nes_params);
    nes->setFramebuff(get_frame_buffer0());
    nes->attach_joy(0, &controller);
  }

  cart = new Cartridge(parseROM(romdata, rom_data_size));

  // Slap a cartridge in!
  nes->loadCartridge(cart->get_mapper());

  // Power-cycle the NES
  nes->power_cycle();

  initialized = true;
}

void run_nes_rom() {
  static float frame_time_total = 0;
  static int num_frames_emulated = 0;
  auto start = std::chrono::high_resolution_clock::now();
  // get input
  get_input_state(&state);
  // set input
  using namespace JOY_Standard_Button;
  controller.set_button(A, state.a);
  controller.set_button(B, state.b);
  controller.set_button(Start, state.start);
  controller.set_button(Select, state.select);
  controller.set_button(Up, state.up);
  controller.set_button(Down, state.down);
  controller.set_button(Left, state.left);
  controller.set_button(Right, state.right);
  // step frame
  nes->step_frame();
  // TODO: audio output
  // TODO: video output
  // frame rate should be 60 FPS, so 1/60th second is what we want to sleep for
  auto end = std::chrono::high_resolution_clock::now();
  frame_time_total += std::chrono::duration<float>(end-start).count();
  if (num_frames_emulated++ == 60) {
    fmt::print("NES FPS: {:2f}\n", (float)num_frames_emulated / frame_time_total);
    num_frames_emulated = 0;
    frame_time_total = 0;
  }
  auto delay = std::chrono::duration<float>(1.0f/60.0f);
  std::this_thread::sleep_until(start + delay);
}

void load_nes(std::string_view save_path) {
}

void save_nes(std::string_view save_path) {
}

void deinit_nes() {
  nes->removeCartridge();
  delete cart;
}
