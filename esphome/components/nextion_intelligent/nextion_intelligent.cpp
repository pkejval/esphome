#include "nextion_intelligent.h"
#include "esphome/core/util.h"
#include <cstdarg>

namespace esphome {
namespace nextion_intelligent {

static const char *const TAG = "nextion_intelligent";

// Nextion end command bytes - required after every command
static const uint8_t NEXTION_END_CMD[3] = {0xFF, 0xFF, 0xFF};

void NextionIntelligent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Nextion Intelligent Display...");

  // Call lambda if set
  if (this->lambda_ != nullptr) {
    this->lambda_(*this);
  }
}

void NextionIntelligent::loop() {
  // Process incoming bytes from Nextion
  while (available()) {
    uint8_t byte = read();
    this->handle_rx_byte_(byte);
  }
}

void NextionIntelligent::handle_rx_byte_(uint8_t byte) {
  if (this->recv_state_ == NextionRecvState::IDLE) {
    // In IDLE state, store the command byte and move to COMMAND state
    this->recv_command_ = byte;
    this->recv_state_ = NextionRecvState::COMMAND;
    this->recv_buffer_pos_ = 0;
    return;
  }

  // We're in COMMAND state, store received byte
  this->recv_buffer_[this->recv_buffer_pos_++] = byte;

  // Check if we've received all three 0xFF bytes (command terminator)
  if (this->recv_buffer_pos_ >= 3 && this->recv_buffer_[0] == 0xFF && this->recv_buffer_[1] == 0xFF &&
      this->recv_buffer_[2] == 0xFF) {
    // Process received command
    if (this->recv_command_ == 0x88 && this->on_boot_callback_ != nullptr) {
      // Boot command (0x88) - trigger the registered callback
      ESP_LOGD(TAG, "Received boot command from Nextion");
      this->on_boot_callback_();
    }

    // Reset state machine for next command
    this->recv_state_ = NextionRecvState::IDLE;
    return;
  }

  // If buffer is full but we haven't received terminator, reset state machine
  if (this->recv_buffer_pos_ >= sizeof(this->recv_buffer_)) {
    this->recv_state_ = NextionRecvState::IDLE;
  }
}

void NextionIntelligent::dump_config() {
  ESP_LOGCONFIG(TAG, "Nextion Intelligent Display:");
  LOG_UART_DEVICE(this);
}

void NextionIntelligent::send_command(const std::string &command) {
  // Send the command string
  write_str(command.c_str());

  // Send the three ending bytes
  write_array(NEXTION_END_CMD, sizeof(NEXTION_END_CMD));

  // Flush UART buffer to ensure command is sent immediately
  flush();

  ESP_LOGV(TAG, "Sent command: %s", command.c_str());
}

void NextionIntelligent::set_component_value(const std::string &component_name, int value) {
  std::string command = component_name + ".val=" + to_string(value);
  send_command(command);
}

void NextionIntelligent::set_component_value(const std::string &component_name, float value) {
  std::string command = component_name + ".val=" + to_string(value);
  send_command(command);
}

void NextionIntelligent::set_component_text(const std::string &component_name, const std::string &text) {
  std::string command = component_name + ".txt=\"" + text + "\"";
  send_command(command);
}

void NextionIntelligent::set_component_text_printf(const std::string &component_name, const char *format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  int ret = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (ret < 0)
    return;

  this->set_component_text(component_name, buffer);
}

void NextionIntelligent::set_component_picture(const std::string &component_name, int picture_id) {
  std::string command = component_name + ".picc=" + to_string(picture_id);
  send_command(command);
}

void NextionIntelligent::set_component_picture1(const std::string &component_name, int picture_id) {
  std::string command = component_name + ".picc1=" + to_string(picture_id);
  send_command(command);
}

void NextionIntelligent::set_component_background_color(const std::string &component_name, int color) {
  std::string command = component_name + ".bco=" + to_string(color);
  send_command(command);
}

void NextionIntelligent::set_component_background_color(const std::string &component_name, Color color) {
  // Convert Color to RGB565 format used by Nextion
  int r5 = (color.r * 31) / 255;
  int g6 = (color.g * 63) / 255;
  int b5 = (color.b * 31) / 255;
  int rgb565 = (r5 << 11) | (g6 << 5) | b5;

  this->set_component_background_color(component_name, rgb565);
}

void NextionIntelligent::set_component_font_color(const std::string &component_name, int color) {
  std::string command = component_name + ".pco=" + to_string(color);
  send_command(command);
}

void NextionIntelligent::set_component_font_color(const std::string &component_name, Color color) {
  // Convert Color to RGB565 format used by Nextion
  int r5 = (color.r * 31) / 255;
  int g6 = (color.g * 63) / 255;
  int b5 = (color.b * 31) / 255;
  int rgb565 = (r5 << 11) | (g6 << 5) | b5;

  this->set_component_font_color(component_name, rgb565);
}

void NextionIntelligent::set_page(int page) {
  std::string command = "page " + to_string(page);
  send_command(command);
  this->current_page_ = page;
}

}  // namespace nextion_intelligent
}  // namespace esphome
