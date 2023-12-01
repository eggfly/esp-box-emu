#pragma once

#include "cart.hpp"
#if defined(ENABLE_SMS)
#include "sms.hpp"
#endif

class SmsCart : public Cart {
public:

  SmsCart(const Cart::Config& config)
    : Cart(config) {
    init();
  }

  ~SmsCart() {
    logger_.info("~SmsCart()");
    deinit();
  }

  virtual void reset() override {
    Cart::reset();
#if defined(ENABLE_SMS)
    reset_sms();
#endif
  }

  virtual void load() override {
    Cart::load();
#if defined(ENABLE_SMS)
    load_sms(get_save_path());
#endif
  }

  virtual void save() override {
    Cart::save();
#if defined(ENABLE_SMS)
    save_sms(get_save_path(true));
#endif
  }

  virtual void init() override {
    Cart::init();

#if defined(ENABLE_SMS)
    switch (info_.platform) {
    case Emulator::SEGA_MASTER_SYSTEM:
      logger_.info("sms::init()");
      init_sms(romdata_, rom_size_bytes_);
      break;
    case Emulator::SEGA_GAME_GEAR:
      logger_.info("gg::init()");
      init_gg(romdata_, rom_size_bytes_);
      break;
    default:
      logger_.error("unknown platform");
      return;
    }
    start_sms_tasks();
#endif
  }

  virtual void deinit() override {
    logger_.info("deinit()");
#if defined(ENABLE_SMS)
    stop_sms_tasks();
    deinit_sms();
#endif
  }

  virtual bool run() override {
#if defined(ENABLE_SMS)
    run_sms_rom();
#endif
    return Cart::run();
  }

protected:
  // SMS
  static constexpr size_t SMS_WIDTH = 256;
  static constexpr size_t SMS_HEIGHT = 192;

  virtual void pre_menu() override {
    Cart::pre_menu();
#if defined(ENABLE_SMS)
    logger_.info("sms::pre_menu()");
    stop_sms_tasks();
#endif
  }

  virtual void post_menu() override {
    Cart::post_menu();
#if defined(ENABLE_SMS)
    logger_.info("sms::post_menu()");
    start_sms_tasks();
#endif
  }

  virtual void set_original_video_setting() override {
#if defined(ENABLE_SMS)
    logger_.info("sms::video: original");
    set_sms_video_original();
#endif
  }

  virtual std::pair<size_t, size_t> get_video_size() const override {
    return std::make_pair(SMS_WIDTH, SMS_HEIGHT);
  }

  virtual std::vector<uint8_t> get_video_buffer() const override {
#if defined(ENABLE_SMS)
    return get_sms_video_buffer();
#else
    return std::vector<uint8_t>();
#endif
  }

  virtual void set_fit_video_setting() override {
#if defined(ENABLE_SMS)
    logger_.info("sms::video: fit");
    set_sms_video_fit();
#endif
  }

  virtual void set_fill_video_setting() override {
#if defined(ENABLE_SMS)
    logger_.info("sms::video: fill");
    set_sms_video_fill();
#endif
  }

  virtual std::string get_save_extension() const override {
    switch (info_.platform) {
    case Emulator::SEGA_MASTER_SYSTEM:
      return "_sms.sav";
    case Emulator::SEGA_GAME_GEAR:
      return "_gg.sav";
    default:
      return Cart::get_save_extension();
    }
  }
};
