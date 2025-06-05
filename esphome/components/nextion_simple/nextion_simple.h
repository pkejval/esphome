#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/color.h"

namespace esphome {
namespace nextion_simple {

class NextionSimple : public Component {
 public:
  NextionSimple();

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_uart_parent(uart::UARTComponent *parent) { this->uart_parent_ = parent; }
  void set_tft_url(const std::string &tft_url) { this->tft_url_ = tft_url; }

  void set_component_value(const std::string &component_name, float value);
  void set_component_text(const std::string &component_name, const std::string &text, const std::vector<std::string> &args = {});
  void set_component_text_printf(const std::string &component_name, const char *format, ...);
  void set_component_picc(const std::string &component_name, int value);
  void set_component_picc1(const std::string &component_name, int value);
  void set_component_background_color(const std::string &component_name, int color);
  void set_component_background_color(const std::string &component_name, Color color);
  void set_component_font_color(const std::string &component_name, int color);
  void set_component_font_color(const std::string &component_name, Color color);
  void set_component_visibility(const std::string &component_name, bool state);
  void set_component_visibility(const std::string &component_name, int state);
  void set_nextion_ready_cooldown(uint32_t cooldown) { nextion_ready_cooldown_ = cooldown; }
  void set_page(int page);
  void goto_page(int page);

  inline void send_command(const char *cmd, size_t len) {
    if (this->upload_in_progress_) {
      return;
    }
    if (len > 256) {
      len = 256;
    }
    char buffer[256 + 3];
    memcpy(buffer, cmd, len);
    buffer[len]     = static_cast<char>(0xFF);
    buffer[len + 1] = static_cast<char>(0xFF);
    buffer[len + 2] = static_cast<char>(0xFF);
    this->uart_parent_->write_array(reinterpret_cast<const uint8_t *>(buffer), len + 3);
  }

  inline void send_command_cstr(const char *format, ...) {
    if (this->upload_in_progress_) {
      return;
    }
    char buf[128];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (n < 0) {
      ESP_LOGE(TAG, "Error formatting command");
      return;
    }
    if (static_cast<size_t>(n) >= sizeof(buf)) {
      ESP_LOGW(TAG, "Command too long (%d bytes), truncating", n);
      n = sizeof(buf) - 1;
    }
    this->send_command(buf, static_cast<size_t>(n));
  }

  void reset_nextion();
  void upload_tft();
  bool is_uploading() const { return this->upload_in_progress_; }
  int get_current_page() const { return this->current_page_; }

  void add_on_setup_callback(std::function<void()> &&callback) {
    this->on_setup_callback_.add(std::move(callback));
  }
  void add_on_page_callback(std::function<void(int)> &&callback) {
    this->on_page_callback_.add(std::move(callback));
  }
  void add_on_nextion_ready_callback(std::function<void()> &&callback) {
    this->on_nextion_ready_callback_.add(std::move(callback));
  }

 protected:
  void process_command(const uint8_t* data, size_t length);

  bool prepare_nextion_for_upload_();
  bool wait_for_nextion_ack_();
  bool send_data_to_nextion_(const uint8_t* data, size_t data_size);
  uint32_t get_free_heap_();

  bool upload_tft_arduino_();
  bool upload_tft_esp_idf_();
  bool upload_end_(bool successful);

  inline int color_to_integer_(Color color) {
    return ((color.r & 0xF8) << 8) | ((color.g & 0xFC) << 3) | (color.b >> 3);
  }

  uart::UARTComponent *uart_parent_{nullptr};
  std::string tft_url_;
  uint32_t content_length_{0};
  bool upload_in_progress_{false};

  static constexpr size_t BUFFER_SIZE = 64;
  uint8_t rx_buffer_[BUFFER_SIZE];
  size_t buffer_index_{0};

  int current_page_{0};
  uint32_t nextion_ready_cooldown_{1000};
  uint32_t last_nextion_ready_time_{0};

  CallbackManager<void()> on_setup_callback_;
  CallbackManager<void(int)> on_page_callback_;
  CallbackManager<void()> on_nextion_ready_callback_;

  static const char *TAG;
};

template<typename... Ts>
class SetComponentValueAction : public Action<Ts...> {
 public:
  SetComponentValueAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_value(std::function<float(Ts...)> fn) { this->value_func_ = fn; }
  void play(Ts... x) override {
    float val = this->value_func_(x...);
    this->parent_->set_component_value(this->component_name_, val);
  }
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  std::function<float(Ts...)> value_func_;
};

template<typename... Ts>
class SetComponentTextAction : public Action<Ts...> {
 public:
  SetComponentTextAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_value(std::function<std::string(Ts...)> fn) { this->value_func_ = fn; }
  void play(Ts... x) override {
    std::string txt = this->value_func_(x...);
    this->parent_->set_component_text(this->component_name_, txt);
  }
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  std::function<std::string(Ts...)> value_func_;
};

template<typename... Ts>
class SetComponentPiccAction : public Action<Ts...> {
 public:
  SetComponentPiccAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_value(int v) { this->value_ = v; }
  void play(Ts... x) override {
    this->parent_->set_component_picc(this->component_name_, this->value_);
  }
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int value_;
};

template<typename... Ts>
class SetComponentPicc1Action : public Action<Ts...> {
 public:
  SetComponentPicc1Action(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_value(int v) { this->value_ = v; }
  void play(Ts... x) override {
    this->parent_->set_component_picc1(this->component_name_, this->value_);
  }
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int value_;
};

template<typename... Ts>
class SetComponentBackgroundColorAction : public Action<Ts...> {
 public:
  SetComponentBackgroundColorAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_color(int c) { this->color_ = c; }
  void play(Ts... x) override {
    this->parent_->set_component_background_color(this->component_name_, this->color_);
  }
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int color_;
};

template<typename... Ts>
class SetComponentBackgroundColorRGBAction : public Action<Ts...> {
 public:
  SetComponentBackgroundColorRGBAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_color(Color c) { this->color_ = c; }
  void play(Ts... x) override {
    this->parent_->set_component_background_color(this->component_name_, this->color_);
  }
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  Color color_;
};

template<typename... Ts>
class SetComponentFontColorAction : public Action<Ts...> {
 public:
  SetComponentFontColorAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_color(int c) { this->color_ = c; }
  void play(Ts... x) override {
    this->parent_->set_component_font_color(this->component_name_, this->color_);
  }
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int color_;
};

template<typename... Ts>
class SetComponentFontColorRGBAction : public Action<Ts...> {
 public:
  SetComponentFontColorRGBAction(NextionSimple *parent) : parent_(parent) {}
  void set_component_name(const std::string &nm) { this->component_name_ = nm; }
  void set_color(Color c) { this->color_ = c; }
  void play(Ts... x) override {
    this->parent_->set_component_font_color(this->component_name_, this->color_);
  }
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  Color color_;
};

template<typename... Ts>
class SetPageAction : public Action<Ts...> {
 public:
  SetPageAction(NextionSimple *parent) : parent_(parent) {}
  void set_page(int p) { this->page_ = p; }
  void play(Ts... x) override {
    this->parent_->set_page(this->page_);
  }
 protected:
  NextionSimple *parent_;
  int page_;
};

template<typename... Ts>
class UploadTftAction : public Action<Ts...> {
 public:
  UploadTftAction(NextionSimple *parent) : parent_(parent) {}
  void play(Ts... x) override {
    this->parent_->upload_tft();
  }
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
