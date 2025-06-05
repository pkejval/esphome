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

  ESP_LOGD(TAG, "Running on_setup callback");
  this->on_setup_callback_.call();
}

void NextionSimple::loop() {
  if (this->upload_in_progress_)
    return;

  static uint8_t buffer[64];
  static size_t buffer_index = 0;

  // Early return if no data available
  size_t available = this->uart_parent_->available();
  if (available == 0)
    return;

  // Batch read data into buffer
  size_t space_available = sizeof(buffer) - buffer_index;
  if (space_available > 0) {
    size_t bytes_to_read = std::min(available, space_available);
    size_t bytes_read = this->uart_parent_->read_array(&buffer[buffer_index], bytes_to_read);
    buffer_index += bytes_read;
  }

  // Process all complete commands in buffer
  size_t pos = 0;
  while (pos + 2 < buffer_index) {
    // Look for command terminator (three consecutive 0xFF bytes)
    if (buffer[pos] == 0xFF && buffer[pos + 1] == 0xFF && buffer[pos + 2] == 0xFF) {
      // Process command data before the terminator
      if (pos > 0) {
        this->process_command(buffer, pos);
      }

      // Move past this command and its terminator
      pos += 3;

      // Shift remaining data to start of buffer
      if (pos < buffer_index) {
        memmove(buffer, buffer + pos, buffer_index - pos);
        buffer_index -= pos;
      } else {
        buffer_index = 0;
      }

      // Reset position for next search
      pos = 0;
    } else {
      // Move to next position
      pos++;
    }
  }

  // Handle buffer overflow - preserve last bytes in case of partial command
  if (buffer_index >= sizeof(buffer) - 3) {
    const size_t keep_bytes = 20;  // Keep enough for potential partial command
    if (buffer_index > keep_bytes) {
      memmove(buffer, buffer + buffer_index - keep_bytes, keep_bytes);
      buffer_index = keep_bytes;
    }
  }
}

void NextionSimple::process_command(const uint8_t *command, size_t length) {
  // Early return for empty commands
  if (length == 0)
    return;

  // Extract the command code (first byte)
  const uint8_t cmd_code = command[0];

  // Fast path processing for common command types
  switch (cmd_code) {
    case 0x66:  // Current page ID
      if (length >= 2) {
        // Store the page number and trigger callback
        this->current_page_ = command[1];
        ESP_LOGD(TAG, "Current page: %d", this->current_page_);
        this->on_page_callback_.call(this->current_page_);
      } else {
        ESP_LOGW(TAG, "Invalid page command (too short)");
      }
      break;

    case 0x88: {  // System startup / ready notification
      const uint32_t current_time = millis();

      // Apply rate limiting to ready callbacks
      if (current_time - this->last_nextion_ready_time_ >= this->nextion_ready_cooldown_) {
        this->last_nextion_ready_time_ = current_time;
        ESP_LOGD(TAG, "Nextion ready - running callback");
        this->on_nextion_ready_callback_.call();
      } else {
        ESP_LOGV(TAG, "Nextion ready (throttled)");
      }
      break;
    }
    default:
      // Log unknown commands
      ESP_LOGD(TAG, "Unknown Nextion command: 0x%02X", cmd_code);
      break;
  }
}

void NextionSimple::dump_config() {
  ESP_LOGCONFIG(TAG, "Nextion Simple:");
  ESP_LOGCONFIG(TAG, "  TFT URL: %s", this->tft_url_.c_str());
}

// Send a command to the Nextion display
void NextionSimple::send_command(std::string_view command) {
  if (this->upload_in_progress_) {
    ESP_LOGW(TAG, "Upload in progress, not sending command: %s", std::string(command).c_str());
    return;
  }

  const uint8_t *data = reinterpret_cast<const uint8_t *>(command.data());
  this->uart_parent_->write_array(data, command.size());
  this->uart_parent_->write_array(NEXTION_CMD_TERMINATOR, sizeof(NEXTION_CMD_TERMINATOR));
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

// Set component value (float)
void NextionSimple::set_component_value(const std::string &component_name, float value) {
  const int integer_value = static_cast<int>(value);
  this->send_command_printf("%s.val=%d", component_name.c_str(), integer_value);
}

// Set component text
void NextionSimple::set_component_text(const std::string &component_name, const std::string &text,
                                       const std::vector<std::string> &args) {
  if (args.empty()) {
    // Simple text setting without formatting
    this->send_command_printf("%s.txt=\"%s\"", component_name.c_str(), text.c_str());
  } else {
    // Text with formatting
    std::string formatted_text = text;
    size_t pos = 0;
    size_t arg_index = 0;

    // Replace all %s placeholders with the corresponding arg
    while ((pos = formatted_text.find("%s", pos)) != std::string::npos && arg_index < args.size()) {
      formatted_text.replace(pos, 2, args[arg_index]);
      pos += args[arg_index].length();
      arg_index++;
    }

    this->send_command_printf("%s.txt=\"%s\"", component_name.c_str(), formatted_text.c_str());
  }
}

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
void NextionSimple::set_page(int page) { this->send_command_printf("page %d", page); }

void NextionSimple::goto_page(int page) { this->set_page(page); }

void NextionSimple::reset_nextion() {
  ESP_LOGI(TAG, "Resetting Nextion...");
  this->send_command("rest");
#ifdef USE_ESP_IDF
  vTaskDelay(pdMS_TO_TICKS(1000));
#else
  delay(1000);  // NOLINT
#endif
  ESP_LOGI(TAG, "Nextion reset completed");
}

}  // namespace nextion_simple
}  // namespace esphome
