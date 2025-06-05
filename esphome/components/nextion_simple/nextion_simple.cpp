#include "nextion_simple.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include <cstdarg>
#include <cstring>

namespace esphome {
namespace nextion_simple {

const char *NextionSimple::TAG = "nextion_simple";

NextionSimple::NextionSimple() {}

void NextionSimple::setup() {
  ESP_LOGCONFIG(TAG, "Initializing Nextion Simple...");
  if (this->uart_parent_ == nullptr) {
    ESP_LOGE(TAG, "UART parent not set!");
    this->mark_failed();
    return;
  }
  this->on_setup_callback_.call();
}

void NextionSimple::loop() {
  if (this->upload_in_progress_) {
    return;
  }
  size_t available = this->uart_parent_->available();
  if (available == 0) {
    return;
  }
  size_t space_left = BUFFER_SIZE - this->buffer_index_;
  if (space_left == 0) {
    static constexpr size_t KEEP = 20;
    if (BUFFER_SIZE > KEEP) {
      memmove(this->rx_buffer_, this->rx_buffer_ + (BUFFER_SIZE - KEEP), KEEP);
      this->buffer_index_ = KEEP;
    }
    space_left = BUFFER_SIZE - this->buffer_index_;
  }
  size_t to_read = (available < space_left) ? available : space_left;
  size_t read_bytes = this->uart_parent_->read_array(this->rx_buffer_ + this->buffer_index_, to_read);
  this->buffer_index_ += read_bytes;
  size_t pos = 0;
  while (pos + 2 < this->buffer_index_) {
    if (this->rx_buffer_[pos] == 0xFF &&
        this->rx_buffer_[pos + 1] == 0xFF &&
        this->rx_buffer_[pos + 2] == 0xFF) {
      if (pos > 0) {
        this->process_command(this->rx_buffer_, pos);
      }
      size_t new_start = pos + 3;
      size_t remain = this->buffer_index_ - new_start;
      if (remain > 0) {
        memmove(this->rx_buffer_, this->rx_buffer_ + new_start, remain);
      }
      this->buffer_index_ = remain;
      pos = 0;
    } else {
      pos++;
    }
  }
}

void NextionSimple::process_command(const uint8_t* data, size_t length) {
  if (length == 0) {
    return;
  }
  uint8_t cmd_code = data[0];
  switch (cmd_code) {
    case 0x66:
      if (length >= 2) {
        int page = data[1];
        this->current_page_ = page;
        ESP_LOGD(TAG, "Current page: %d", page);
        this->on_page_callback_.call(page);
      } else {
        ESP_LOGW(TAG, "Page data too short");
      }
      break;
    case 0x88: {
      uint32_t now = millis();
      if (now - this->last_nextion_ready_time_ >= this->nextion_ready_cooldown_) {
        this->last_nextion_ready_time_ = now;
        ESP_LOGD(TAG, "Nextion ready, triggering callback");
        this->on_nextion_ready_callback_.call();
      } else {
        ESP_LOGV(TAG, "Nextion ready (throttled)");
      }
      break;
    }
    default:
      ESP_LOGD(TAG, "Unknown Nextion command: 0x%02X", cmd_code);
      break;
  }
}

void NextionSimple::dump_config() {
  ESP_LOGCONFIG(TAG, "Nextion Simple config:");
  ESP_LOGCONFIG(TAG, "  TFT URL: %s", this->tft_url_.c_str());
}

void NextionSimple::set_component_value(const std::string &component_name, float value) {
  int iv = static_cast<int>(value);
  this->send_command_cstr("%s.val=%d", component_name.c_str(), iv);
}

void NextionSimple::set_component_text(const std::string &component_name, const std::string &text, const std::vector<std::string> &args) {
  if (args.empty()) {
    this->send_command_cstr("%s.txt=\"%s\"", component_name.c_str(), text.c_str());
  } else {
    std::string result;
    result.reserve(text.size() + args.size() * 10);
    size_t last = 0;
    for (const auto &arg : args) {
      size_t pos = text.find("%s", last);
      if (pos == std::string::npos) break;
      result.append(text.data() + last, pos - last);
      result.append(arg);
      last = pos + 2;
    }
    result.append(text.data() + last, text.size() - last);
    this->send_command_cstr("%s.txt=\"%s\"", component_name.c_str(), result.c_str());
  }
}

void NextionSimple::set_component_text_printf(const std::string &component_name, const char *format, ...) {
  char buf[128];
  va_list args;
  va_start(args, format);
  int n = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  if (n < 0 || static_cast<size_t>(n) >= sizeof(buf)) {
    ESP_LOGE(TAG, "Error formatting text");
    return;
  }
  this->set_component_text(component_name, buf);
}

void NextionSimple::set_component_picc(const std::string &component_name, int value) {
  this->send_command_cstr("%s.picc=%d", component_name.c_str(), value);
}

void NextionSimple::set_component_picc1(const std::string &component_name, int value) {
  this->send_command_cstr("%s.picc1=%d", component_name.c_str(), value);
}

void NextionSimple::set_component_background_color(const std::string &component_name, int color) {
  this->send_command_cstr("%s.bco=%d", component_name.c_str(), color);
}

void NextionSimple::set_component_background_color(const std::string &component_name, Color color) {
  int col = this->color_to_integer_(color);
  this->send_command_cstr("%s.bco=%d", component_name.c_str(), col);
}

void NextionSimple::set_component_font_color(const std::string &component_name, int color) {
  this->send_command_cstr("%s.pco=%d", component_name.c_str(), color);
}

void NextionSimple::set_component_font_color(const std::string &component_name, Color color) {
  int col = this->color_to_integer_(color);
  this->send_command_cstr("%s.pco=%d", component_name.c_str(), col);
}

void NextionSimple::set_component_visibility(const std::string &component_name, bool state) {
  this->set_component_visibility(component_name, (int)state);
}

void NextionSimple::set_component_visibility(const std::string &component_name, int state) {
  this->send_command_cstr("%s.vis=%d", component_name.c_str(), state);
}

void NextionSimple::set_page(int page) {
  this->send_command_cstr("page %d", page);
}

void NextionSimple::goto_page(int page) {
  this->set_page(page);
}

void NextionSimple::reset_nextion() {
  ESP_LOGI(TAG, "Reset Nextion...");
  const char reset_cmd[] = "rest";
  this->send_command(reset_cmd, sizeof(reset_cmd) - 1);
#ifdef USE_ESP_IDF
  vTaskDelay(pdMS_TO_TICKS(1000));
#else
  delay(1000);
#endif
  ESP_LOGI(TAG, "Nextion reset complete");
}

void NextionSimple::upload_tft() {
  this->upload_in_progress_ = true;
#ifdef USE_ARDUINO
  this->upload_tft_arduino_();
#elif defined(USE_ESP_IDF)
  this->upload_tft_esp_idf_();
#endif
}

uint32_t NextionSimple::get_free_heap_() {
#if defined(USE_ESP32)
#ifdef USE_ESP_IDF
  return esp_get_free_heap_size();
#else
  return ESP.getHeapSize();
#endif
#elif defined(USE_ESP8266)
  return ESP.getFreeHeap();
#else
  return 0;
#endif
}

}  // namespace nextion_simple
}  // namespace esphome
