#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/color.h"
#include <string_view>

#ifdef USE_ESP_IDF
#include "esp_http_client.h"
#endif

namespace esphome {
namespace nextion_simple {

static constexpr uint8_t NEXTION_CMD_TERMINATOR[3] = {0xFF, 0xFF, 0xFF};

class NextionSimple : public Component {
 public:
  // Constructor
  NextionSimple();

  // Set UART
  void set_uart_parent(uart::UARTComponent *parent) { this->uart_parent_ = parent; }

  // Set TFT URL
  void set_tft_url(const std::string &tft_url) { this->tft_url_ = tft_url; }

  // Component interface
  void setup() override;
  void HOT loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  // Command functions
  void set_component_value(const std::string &component_name, float value);
  void set_component_text(const std::string &component_name, const std::string &text,
                          const std::vector<std::string> &args = {});
  void HOT set_component_text_printf(const std::string &component_name, const char *format, ...)
      __attribute__((format(printf, 3, 4)));
  void set_component_picc(const std::string &component_name, int value);
  void set_component_picc1(const std::string &component_name, int value);
  void set_component_background_color(const std::string &component_name, int color);
  void set_component_background_color(const std::string &component_name, Color color);
  void set_component_font_color(const std::string &component_name, int color);
  void set_component_font_color(const std::string &component_name, Color color);
  void set_nextion_ready_cooldown(uint32_t cooldown) { nextion_ready_cooldown_ = cooldown; }
  void set_page(int page);
  void goto_page(int page);
  // Send command functions
  void HOT send_command(std::string_view command);
  void HOT send_command_printf(const char *format, ...) __attribute__((format(printf, 2, 3)));
  void reset_nextion();
  // TFT update
  void upload_tft();
  bool is_uploading() const { return this->upload_in_progress_; }

  int get_current_page() const { return this->current_page_; }

  // Callback
  void add_on_setup_callback(std::function<void()> &&callback) { this->on_setup_callback_.add(std::move(callback)); }

  void add_on_page_callback(std::function<void(int)> &&callback) { this->on_page_callback_.add(std::move(callback)); }

  void add_on_nextion_ready_callback(std::function<void()> &&callback) {
    this->on_nextion_ready_callback_.add(std::move(callback));
  }

 protected:
  void HOT process_command(const uint8_t *command, size_t length);
  int current_page_ = 0;
  uint32_t nextion_ready_cooldown_;
  uint32_t last_nextion_ready_time_{0};

  // Helper for color conversion
  static constexpr int color_to_integer_(Color color) {
    return ((color.r & 0xF8) << 8) | ((color.g & 0xFC) << 3) | (color.b >> 3);
  }

  // TFT upload helpers
  void upload_tft_arduino_();
  void upload_tft_esp_idf_();
  bool prepare_nextion_for_upload_();
  bool wait_for_nextion_ack_();
  bool send_data_to_nextion_(const uint8_t *data, size_t data_size);
  uint32_t get_free_heap_();

  // UART parent
  uart::UARTComponent *uart_parent_{nullptr};

  // TFT URL
  std::string tft_url_;
  uint32_t content_length_{0};

#ifdef USE_ARDUINO
  int upload_by_chunks_(HTTPClient &http_client, uint32_t &range_start);
  inline uint32_t get_free_heap_();
#endif

#ifdef USE_ESP_IDF
  int upload_by_chunks_(esp_http_client_handle_t http_client, uint32_t &range_start);
#endif

  bool upload_end_(bool successful);
  bool upload_in_progress_{false};

  // Callback
  CallbackManager<void()> on_setup_callback_;
  CallbackManager<void(int)> on_page_callback_;
  CallbackManager<void()> on_nextion_ready_callback_;
};

template<typename... Ts> class SetComponentValueAction : public Action<Ts...> {
 public:
  SetComponentValueAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_value(std::function<float(Ts...)> value_func) { this->value_func_ = value_func; }
  void play(Ts... x) override {
    float value = this->value_func_(x...);
    this->parent_->set_component_value(this->component_name_, value);
  }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  std::function<float(Ts...)> value_func_;
};

template<typename... Ts> class SetComponentTextAction : public Action<Ts...> {
 public:
  SetComponentTextAction(NextionSimple *parent) : parent_(parent) {}

  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_value(std::function<std::string(Ts...)> value_func) { this->value_func_ = value_func; }

  void play(Ts... x) override {
    std::string value = this->value_func_(x...);
    this->parent_->set_component_text(this->component_name_, value);
  }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  std::function<std::string(Ts...)> value_func_;
};

// Action to set component picc
template<typename... Ts> class SetComponentPiccAction : public Action<Ts...> {
 public:
  SetComponentPiccAction(NextionSimple *parent) : parent_(parent) {}

  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_value(int value) { this->value_ = value; }

  void play(Ts... x) override { this->parent_->set_component_picc(this->component_name_, this->value_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int value_;
};

// Action to set component picc1
template<typename... Ts> class SetComponentPicc1Action : public Action<Ts...> {
 public:
  SetComponentPicc1Action(NextionSimple *parent) : parent_(parent) {}

  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_value(int value) { this->value_ = value; }

  void play(Ts... x) override { this->parent_->set_component_picc1(this->component_name_, this->value_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int value_;
};

// Action to set component background color
template<typename... Ts> class SetComponentBackgroundColorAction : public Action<Ts...> {
 public:
  SetComponentBackgroundColorAction(NextionSimple *parent) : parent_(parent) {}

  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_color(int color) { this->color_ = color; }

  void play(Ts... x) override { this->parent_->set_component_background_color(this->component_name_, this->color_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int color_;
};

// Action to set component background color (RGB)
template<typename... Ts> class SetComponentBackgroundColorRGBAction : public Action<Ts...> {
 public:
  SetComponentBackgroundColorRGBAction(NextionSimple *parent) : parent_(parent) {}

  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_color(Color color) { this->color_ = color; }

  void play(Ts... x) override { this->parent_->set_component_background_color(this->component_name_, this->color_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  Color color_;
};

// Action to set component font color
template<typename... Ts> class SetComponentFontColorAction : public Action<Ts...> {
 public:
  SetComponentFontColorAction(NextionSimple *parent) : parent_(parent) {}

  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_color(int color) { this->color_ = color; }

  void play(Ts... x) override { this->parent_->set_component_font_color(this->component_name_, this->color_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int color_;
};

// Action to set component font color (RGB)
template<typename... Ts> class SetComponentFontColorRGBAction : public Action<Ts...> {
 public:
  SetComponentFontColorRGBAction(NextionSimple *parent) : parent_(parent) {}

  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_color(Color color) { this->color_ = color; }

  void play(Ts... x) override { this->parent_->set_component_font_color(this->component_name_, this->color_); }

 protected:
  NextionSimple *parent_;
  std::string component_name_;
  Color color_;
};

// Action to set page
template<typename... Ts> class SetPageAction : public Action<Ts...> {
 public:
  SetPageAction(NextionSimple *parent) : parent_(parent) {}

  void set_page(int page) { this->page_ = page; }

  void play(Ts... x) override { this->parent_->set_page(this->page_); }

 protected:
  NextionSimple *parent_;
  int page_;
};

// Action to upload TFT
template<typename... Ts> class UploadTftAction : public Action<Ts...> {
 public:
  UploadTftAction(NextionSimple *parent) : parent_(parent) {}

  void play(Ts... x) override { this->parent_->upload_tft(); }

 protected:
  NextionSimple *parent_;
};

class NextionSetupTrigger : public Trigger<> {
 public:
  explicit NextionSetupTrigger(NextionSimple *parent) {
    parent->add_on_setup_callback([this]() { this->trigger(); });
  }
};

class NextionPageTrigger : public Trigger<int> {
 public:
  explicit NextionPageTrigger(NextionSimple *parent) {
    parent->add_on_page_callback([this](int page) { this->trigger(page); });
  }
};

class NextionReadyTrigger : public Trigger<> {
 public:
  explicit NextionReadyTrigger(NextionSimple *parent) {
    parent->add_on_nextion_ready_callback([this]() { this->trigger(); });
  }
};

}  // namespace nextion_simple
}  // namespace esphome
