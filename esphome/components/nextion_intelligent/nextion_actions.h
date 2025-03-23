#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/color.h"
#include <vector>

namespace esphome {
namespace nextion_intelligent {

class NextionIntelligent;

/**
 * @brief Action to set a component value
 */
template<typename... Ts>
class SetComponentValueAction : public Action<Ts...> {
 public:
  explicit SetComponentValueAction(NextionIntelligent *parent) : parent_(parent) {}
  
  void set_component_value(const std::string &component_name, float value) {
    this->component_name_ = component_name;
    this->value_ = value;
  }
  
  void play(Ts... x) override {
    float value = this->value_.value(x...);
    if (std::trunc(value) == value) {
      // Value is an integer
      int int_value = static_cast<int>(value);
      this->parent_->set_component_value(this->component_name_, int_value);
    } else {
      // Value is a float
      this->parent_->set_component_value(this->component_name_, value);
    }
  }
  
 protected:
  NextionIntelligent *parent_;
  std::string component_name_{};
  TemplatableValue<float, Ts...> value_{};
};

/**
 * @brief Action to set component text
 */
template<typename... Ts>
class SetComponentTextAction : public Action<Ts...> {
 public:
  explicit SetComponentTextAction(NextionIntelligent *parent) : parent_(parent) {}
  
  void set_component_text(const std::string &component_name, const std::string &text) {
    this->component_name_ = component_name;
    this->text_ = text;
  }
  
  void play(Ts... x) override {
    this->parent_->set_component_text(this->component_name_, this->text_.value(x...));
  }
  
 protected:
  NextionIntelligent *parent_;
  std::string component_name_{};
  TemplatableValue<std::string, Ts...> text_{};
};

/**
 * @brief Action to set component text with printf-style formatting
 */
template<typename... Ts>
class SetComponentTextPrintfAction : public Action<Ts...> {
 public:
  explicit SetComponentTextPrintfAction(NextionIntelligent *parent) : parent_(parent) {}
  
  void set_component_name(const std::string &component_name) {
    this->component_name_ = component_name;
  }
  
  void set_format(const std::string &format) {
    this->format_ = format;
  }
  
  void add_argument(std::function<float(Ts...)> arg) {
    this->args_.push_back(arg);
  }
  
  void play(Ts... x) override {
    std::vector<float> values;
    values.reserve(this->args_.size());
    for (const auto &arg : this->args_) {
      values.push_back(arg(x...));
    }
    
    // Format the text with variable number of arguments
    char buffer[256];
    int ret;
    switch (values.size()) {
      case 0:
        ret = snprintf(buffer, sizeof(buffer), this->format_.c_str());
        break;
      case 1:
        ret = snprintf(buffer, sizeof(buffer), this->format_.c_str(), values[0]);
        break;
      case 2:
        ret = snprintf(buffer, sizeof(buffer), this->format_.c_str(), values[0], values[1]);
        break;
      case 3:
        ret = snprintf(buffer, sizeof(buffer), this->format_.c_str(), values[0], values[1], values[2]);
        break;
      case 4:
        ret = snprintf(buffer, sizeof(buffer), this->format_.c_str(), values[0], values[1], values[2], values[3]);
        break;
      case 5:
        ret = snprintf(buffer, sizeof(buffer), this->format_.c_str(), values[0], values[1], values[2], values[3], values[4]);
        break;
      default:
        ESP_LOGW("nextion_intelligent", "Too many arguments for text_printf");
        return;
    }
    
    if (ret < 0)
      return;
    
    this->parent_->set_component_text(this->component_name_, buffer);
  }
  
 protected:
  NextionIntelligent *parent_;
  std::string component_name_{};
  std::string format_{};
  std::vector<std::function<float(Ts...)>> args_{};
};

/**
 * @brief Action to set component picture
 */
template<typename... Ts>
class SetComponentPictureAction : public Action<Ts...> {
 public:
  explicit SetComponentPictureAction(NextionIntelligent *parent) : parent_(parent) {}
  
  void set_component_picture(const std::string &component_name, int picture_id) {
    this->component_name_ = component_name;
    this->picture_id_ = picture_id;
  }
  
  void play(Ts... x) override {
    this->parent_->set_component_picture(this->component_name_, this->picture_id_.value(x...));
  }
  
 protected:
  NextionIntelligent *parent_;
  std::string component_name_{};
  TemplatableValue<int, Ts...> picture_id_{};
};

/**
 * @brief Action to set component picture1
 */
template<typename... Ts>
class SetComponentPicture1Action : public Action<Ts...> {
 public:
  explicit SetComponentPicture1Action(NextionIntelligent *parent) : parent_(parent) {}
  
  void set_component_picture1(const std::string &component_name, int picture_id) {
    this->component_name_ = component_name;
    this->picture_id_ = picture_id;
  }
  
  void play(Ts... x) override {
    this->parent_->set_component_picture1(this->component_name_, this->picture_id_.value(x...));
  }
  
 protected:
  NextionIntelligent *parent_;
  std::string component_name_{};
  TemplatableValue<int, Ts...> picture_id_{};
};

/**
 * @brief Action to set component background color
 */
template<typename... Ts>
class SetComponentBackgroundColorAction : public Action<Ts...> {
 public:
  explicit SetComponentBackgroundColorAction(NextionIntelligent *parent) : parent_(parent) {}
  
  void set_component_background_color(const std::string &component_name, Color color) {
    this->component_name_ = component_name;
    this->color_ = color;
  }
  
  void play(Ts... x) override {
    this->parent_->set_component_background_color(this->component_name_, this->color_.value(x...));
  }
  
 protected:
  NextionIntelligent *parent_;
  std::string component_name_{};
  TemplatableValue<Color, Ts...> color_{};
};

/**
 * @brief Action to set component font color
 */
template<typename... Ts>
class SetComponentFontColorAction : public Action<Ts...> {
 public:
  explicit SetComponentFontColorAction(NextionIntelligent *parent) : parent_(parent) {}
  
  void set_component_font_color(const std::string &component_name, Color color) {
    this->component_name_ = component_name;
    this->color_ = color;
  }
  
  void play(Ts... x) override {
    this->parent_->set_component_font_color(this->component_name_, this->color_.value(x...));
  }
  
 protected:
  NextionIntelligent *parent_;
  std::string component_name_{};
  TemplatableValue<Color, Ts...> color_{};
};

/**
 * @brief Action to set page
 */
template<typename... Ts>
class SetPageAction : public Action<Ts...> {
 public:
  explicit SetPageAction(NextionIntelligent *parent) : parent_(parent) {}
  
  void set_page(int page) {
    this->page_ = page;
  }
  
  void play(Ts... x) override {
    this->parent_->set_page(this->page_.value(x...));
  }
  
 protected:
  NextionIntelligent *parent_;
  TemplatableValue<int, Ts...> page_{};
};

/**
 * @brief Trigger for Nextion boot events
 */
class NextionBootTrigger : public Trigger<> {
 public:
  explicit NextionBootTrigger(NextionIntelligent *parent) {
    parent->add_on_boot_callback([this]() { this->trigger(); });
  }
};

} // namespace nextion_intelligent
} // namespace esphome
