#include "battery.hpp"

static std::shared_ptr<espp::Max1704x> battery_{nullptr};
static std::unique_ptr<espp::Task> battery_task_;
static bool battery_initialized_ = false;

using namespace std::chrono_literals;

void hal::battery_init() {
#if CONFIG_HARDWARE_V1
  if (battery_initialized_) {
    return;
  }
  fmt::print("Initializing battery...\n");
  auto i2c = hal::get_external_i2c();
  bool can_communicate = i2c->probe_device(espp::Max1704x::DEFAULT_ADDRESS);
  if (!can_communicate) {
    fmt::print("Could not communicate with battery!\n");
    // go ahead and set the battery_initialized_ flag to true so we don't try to
    // initialize the battery again
    battery_initialized_ = true;
    return;
  }
  battery_ = std::make_shared<espp::Max1704x>(espp::Max1704x::Config{
      .device_address = espp::Max1704x::DEFAULT_ADDRESS,
      .write = std::bind(&espp::I2c::write, i2c.get(), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, i2c.get(), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    });
  // NOTE: the MAX17048 is tied to the VBAT for its power supply (as you would
  // imagine), this means that we cannnot communicate with it if the battery is
  // not connected. Therefore, if we are unable to communicate with the battery
  // we will just return and not start the battery task.
  battery_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .name = "battery",
        .callback = [](auto &m, auto &cv) {
          // sleep up here so we can easily early return below
          {
            std::unique_lock<std::mutex> lk(m);
            cv.wait_for(lk, 1000ms);
          }
          // get the state of charge (%)
          std::error_code ec;
          auto soc = battery_->get_battery_percentage(ec);
          if (ec) {
            return false;
          }
          // get the charge rate (+/- % per hour)
          auto charge_rate = battery_->get_battery_charge_rate(ec);
          if (ec) {
            return false;
          }
          // now publish a BatteryInfo struct to the battery_topic
          auto battery_info = BatteryInfo{
            .level = soc,
            .charge_rate = charge_rate,
          };
          std::vector<uint8_t> battery_info_data;
          // fmt::print("Publishing battery info: {}\n", battery_info);
          auto bytes_serialized = espp::serialize(battery_info, battery_info_data);
          if (bytes_serialized == 0) {
            return false;
          }
          espp::EventManager::get().publish(battery_topic, battery_info_data);
          return false;
        },
        .stack_size_bytes = 4 * 1024});
  battery_task_->start();
  battery_initialized_ = true;
#else
  fmt::print("Battery not supported on this hardware version!\n");
#endif
}

std::shared_ptr<espp::Max1704x> hal::get_battery() {
  battery_init();
  return battery_;
}
