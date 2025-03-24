#include "nextion_simple.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include <cstring>
#include <cstdarg>

namespace esphome {
namespace nextion_simple {

static const char *TAG = "nextion_simple";

// Constructor
NextionSimple::NextionSimple() {}

void NextionSimple::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Nextion Simple...");
  
  if (this->uart_parent_ == nullptr) {
    ESP_LOGE(TAG, "UART parent is not set!");
    this->mark_failed();
    return;
  }
}

void NextionSimple::loop() {
  if (this->upload_in_progress_)
    return;

  static uint8_t buffer[64] = {0};  // Increased buffer size
  static uint8_t buffer_index = 0;
  static const uint8_t command_terminator[] = {0xFF, 0xFF, 0xFF};

  while (this->uart_parent_->available()) {
    uint8_t byte;
    if (this->uart_parent_->read_byte(&byte)) {
      buffer[buffer_index++] = byte;

      // Check for command terminator
      if (buffer_index >= 3 && memcmp(&buffer[buffer_index - 3], command_terminator, 3) == 0) {
        // Process the complete command
        this->process_command(buffer, buffer_index - 3);
        buffer_index = 0;  // Reset buffer for next command
      }

      // Prevent buffer overflow
      if (buffer_index >= sizeof(buffer)) {
        buffer_index = 0;  // Reset if buffer is full without finding terminator
      }
    }
  }
}

void NextionSimple::process_command(const uint8_t* command, size_t length) {
  // Example command processing
  if (length > 0) {
    switch (command[0]) {
      case 0x66:  // Current page ID
        this->current_page_ = command[1];
        ESP_LOGD(TAG, "Current page: %d", this->current_page_);
        this->on_page_callback_.call(this->current_page_);
        break;
      case 0x88:  // System startup
        ESP_LOGD(TAG, "Received setup command from Nextion");
        this->on_setup_callback_.call();
        break;
      // Add more command handlers here
      default:
        ESP_LOGW(TAG, "Unknown command received: 0x%02X", command[0]);
        break;
    }
  }
}

void NextionSimple::dump_config() {
  ESP_LOGCONFIG(TAG, "Nextion Simple:");
  ESP_LOGCONFIG(TAG, "  TFT URL: %s", this->tft_url_.c_str());
}

// Send a command to the Nextion display
void NextionSimple::send_command(const std::string &command) {
  if (this->upload_in_progress_) {
    ESP_LOGW(TAG, "Upload in progress, not sending command: %s", command.c_str());
    return;
  }

  static char buffer[256];
  size_t command_length = command.length();
  if (command_length + 3 <= sizeof(buffer)) {
    memcpy(buffer, command.c_str(), command_length);
    buffer[command_length] = '\xFF';
    buffer[command_length + 1] = '\xFF';
    buffer[command_length + 2] = '\xFF';
    this->uart_parent_->write_array((const uint8_t *)buffer, command_length + 3);
  } else {
    // Fallback for longer commands
    std::string full_command = command + "\xFF\xFF\xFF";
    this->uart_parent_->write_array((const uint8_t *)full_command.c_str(), full_command.length());
  }
}

// Send a formatted command to the Nextion display
void NextionSimple::send_command_printf(const char *format, ...) {
  // Guard to prevent sending commands during upload
  if (this->upload_in_progress_) {
    ESP_LOGW(TAG, "Upload in progress, not sending formatted command");
    return;
  }
  
  // Format the command
  char buffer[256];
  va_list args;
  va_start(args, format);
  int ret = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  if (ret < 0) {
    ESP_LOGE(TAG, "Error formatting command");
    return;
  }
  
  if (ret >= sizeof(buffer)) {
    ESP_LOGE(TAG, "Command too long for buffer");
    return;
  }
  
  // Send the formatted command
  this->send_command(buffer);
}

// Set component value (integer)
void NextionSimple::set_component_value(const std::string &component_name, int value) {
  this->send_command_printf("%s.val=%d", component_name.c_str(), value);
}

// Set component value (float)
void NextionSimple::set_component_value(const std::string &component_name, float value) {
  const int integer_value = static_cast<int>(value);
  this->send_command_printf("%s.val=%d", component_name.c_str(), integer_value);
}

// Set component text
void NextionSimple::set_component_text(const std::string &component_name, const std::string &text) {
  // Escape quotes in text
  std::string escaped_text = text;
  size_t pos = 0;
  while ((pos = escaped_text.find("\"", pos)) != std::string::npos) {
    escaped_text.replace(pos, 1, "\\\"");
    pos += 2;
  }
  
  this->send_command_printf("%s.txt=\"%s\"", component_name.c_str(), escaped_text.c_str());
}

// Set component text with formatting
void NextionSimple::set_component_text_printf(const std::string &component_name, const char *format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  int ret = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  if (ret < 0) {
    ESP_LOGE(TAG, "Error formatting text");
    return;
  }
  
  if (ret >= sizeof(buffer)) {
    ESP_LOGE(TAG, "Text too long for buffer");
    return;
  }
  
  this->set_component_text(component_name, buffer);
}

// Set component picc value
void NextionSimple::set_component_picc(const std::string &component_name, int value) {
  this->send_command_printf("%s.picc=%d", component_name.c_str(), value);
}

// Set component picc1 value
void NextionSimple::set_component_picc1(const std::string &component_name, int value) {
  this->send_command_printf("%s.picc1=%d", component_name.c_str(), value);
}

// Convert ESPHome Color to Nextion color (RGB565)
inline int NextionSimple::color_to_integer_(Color color) {
  return ((color.r & 0xF8) << 8) | ((color.g & 0xFC) << 3) | (color.b >> 3);
}

// Set component background color (integer)
void NextionSimple::set_component_background_color(const std::string &component_name, int color) {
  this->send_command_printf("%s.bco=%d", component_name.c_str(), color);
}

// Set component background color (Color)
void NextionSimple::set_component_background_color(const std::string &component_name, Color color) {
  int color_value = this->color_to_integer_(color);
  this->set_component_background_color(component_name, color_value);
}

// Set component font color (integer)
void NextionSimple::set_component_font_color(const std::string &component_name, int color) {
  this->send_command_printf("%s.pco=%d", component_name.c_str(), color);
}

// Set component font color (Color)
void NextionSimple::set_component_font_color(const std::string &component_name, Color color) {
  int color_value = this->color_to_integer_(color);
  this->set_component_font_color(component_name, color_value);
}

// Change Nextion page
void NextionSimple::set_page(int page) {
  this->send_command_printf("page %d", page);
}

void NextionSimple::goto_page(int page) {
  this->set_page(page);
}

// Upload TFT file to Nextion
void NextionSimple::upload_tft() {
  if (this->tft_url_.empty()) {
    ESP_LOGE(TAG, "TFT URL is not set");
    return;
  }
  
  if (this->upload_in_progress_) {
    ESP_LOGW(TAG, "Upload already in progress");
    return;
  }
  
  ESP_LOGI(TAG, "Starting TFT upload from URL: %s", this->tft_url_.c_str());
  this->upload_in_progress_ = true;
  
#ifdef USE_ARDUINO
  this->upload_tft_arduino_();
#else
  this->upload_tft_esp_idf_();
#endif
}

// Upload TFT file using Arduino framework
void NextionSimple::upload_tft_arduino_() {}

// Upload TFT file using ESP-IDF framework
void NextionSimple::upload_tft_esp_idf_() {}

void NextionSimple::reset_nextion_() {
  ESP_LOGI(TAG, "Resetting Nextion...");
  this->send_command("rest");
  delay(1000);
  this->upload_in_progress_ = false;
  ESP_LOGI(TAG, "Nextion reset completed");
}

}  // namespace nextion_simple
}  // namespace esphome
