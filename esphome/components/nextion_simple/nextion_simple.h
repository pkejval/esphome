#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/color.h"

namespace esphome {
namespace nextion_simple {

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
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }
  
  // Command functions
  void set_component_value(const std::string &component_name, int value);
  void set_component_value(const std::string &component_name, float value);
  void set_component_text(const std::string &component_name, const std::string &text);
  void set_component_text_printf(const std::string &component_name, const char *format, ...);
  void set_component_picc(const std::string &component_name, int value);
  void set_component_picc1(const std::string &component_name, int value);
  void set_component_background_color(const std::string &component_name, int color);
  void set_component_background_color(const std::string &component_name, Color color);
  void set_component_font_color(const std::string &component_name, int color);
  void set_component_font_color(const std::string &component_name, Color color);
  void set_page(int page);
  void goto_page(int page);
  // Send command functions
  void send_command(const std::string &command);
  void send_command_printf(const char *format, ...);
  void reset_nextion();
  // TFT update
  void upload_tft();
  bool is_uploading() const { return this->upload_in_progress_; }
  
  // Callback
  void add_on_setup_callback(std::function<void()> &&callback) {
    this->on_setup_callback_.add(std::move(callback));
  }
  
 protected:
  
  // Helper for color conversion
  int color_to_integer_(Color color);
  
  // TFT upload helpers
  void upload_tft_arduino_();
  void upload_tft_esp_idf_();
  void reset_nextion_();
  
  // UART parent
  uart::UARTComponent *uart_parent_{nullptr};
  
  // TFT URL
  std::string tft_url_;
  
  // Upload state
  bool upload_in_progress_{false};
  
  // Callback
  CallbackManager<void()> on_setup_callback_;
};

// Action to set component value (int)
template<typename... Ts> class SetComponentValueAction : public Action<Ts...> {
 public:
  SetComponentValueAction(NextionSimple *parent) : parent_(parent) {}
  
  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_value(int value) { this->value_ = value; }
  
  void play(Ts... x) override {
    this->parent_->set_component_value(this->component_name_, this->value_);
  }
  
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  int value_;
};

// Action to set component value (float)
template<typename... Ts> class SetComponentFloatValueAction : public Action<Ts...> {
 public:
  SetComponentFloatValueAction(NextionSimple *parent) : parent_(parent) {}
  
  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_value(float value) { this->value_ = value; }
  
  void play(Ts... x) override {
    this->parent_->set_component_value(this->component_name_, this->value_);
  }
  
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  float value_;
};

// Action to set component text
template<typename... Ts> class SetComponentTextAction : public Action<Ts...> {
 public:
  SetComponentTextAction(NextionSimple *parent) : parent_(parent) {}
  
  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_text(const std::string &text) { this->text_ = text; }
  
  void play(Ts... x) override {
    this->parent_->set_component_text(this->component_name_, this->text_);
  }
  
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  std::string text_;
};

// Action to set component text with printf formatting
template<typename... Ts> class SetComponentTextPrintfAction : public Action<Ts...> {
 public:
  SetComponentTextPrintfAction(NextionSimple *parent) : parent_(parent) {}
  
  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_format(const std::string &format) { this->format_ = format; }
  void set_args(const std::vector<TemplatableValue<Ts...>> &args) { this->args_ = args; }
  
  void play(Ts... x) override {
    // Format string with args
    std::string text = this->format_;
    
    // Process args for formatting if provided
    if (!this->args_.empty()) {
      std::vector<std::string> rendered_args;
      for (const auto &arg : this->args_) {
        rendered_args.push_back(arg.value(x...));
      }
      
      // Simple formatting to replace %s with args
      size_t pos = 0;
      size_t arg_index = 0;
      
      while ((pos = text.find("%s", pos)) != std::string::npos && arg_index < rendered_args.size()) {
        text.replace(pos, 2, rendered_args[arg_index++]);
        pos += rendered_args[arg_index - 1].length();
      }
    }
    
    this->parent_->set_component_text(this->component_name_, text);
  }
  
 protected:
  NextionSimple *parent_;
  std::string component_name_;
  std::string format_;
  std::vector<TemplatableValue<Ts...>> args_;
};

// Action to set component picc
template<typename... Ts> class SetComponentPiccAction : public Action<Ts...> {
 public:
  SetComponentPiccAction(NextionSimple *parent) : parent_(parent) {}
  
  void set_component_name(const std::string &component_name) { this->component_name_ = component_name; }
  void set_value(int value) { this->value_ = value; }
  
  void play(Ts... x) override {
    this->parent_->set_component_picc(this->component_name_, this->value_);
  }
  
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
  
  void play(Ts... x) override {
    this->parent_->set_component_picc1(this->component_name_, this->value_);
  }
  
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
  
  void play(Ts... x) override {
    this->parent_->set_component_background_color(this->component_name_, this->color_);
  }
  
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
  
  void play(Ts... x) override {
    this->parent_->set_component_background_color(this->component_name_, this->color_);
  }
  
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
  
  void play(Ts... x) override {
    this->parent_->set_component_font_color(this->component_name_, this->color_);
  }
  
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
  
  void play(Ts... x) override {
    this->parent_->set_component_font_color(this->component_name_, this->color_);
  }
  
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
  
  void play(Ts... x) override {
    this->parent_->set_page(this->page_);
  }
  
 protected:
  NextionSimple *parent_;
  int page_;
};

// Action to upload TFT
template<typename... Ts> class UploadTftAction : public Action<Ts...> {
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

}  // namespace nextion_simple
}  // namespace esphome
