#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/color.h"
#include "esphome/core/log.h"

namespace esphome {
namespace nextion_intelligent {

// Forward declarations
class NextionBootTrigger;

/**
 * @brief Main class for controlling Nextion Intelligent displays
 * 
 * This component provides high-performance, one-way communication 
 * to a Nextion Intelligent display over UART.
 */
class NextionIntelligent : public Component, public uart::UARTDevice {
 public:
  NextionIntelligent() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }
  
  // Value control functions
  void set_component_value(const std::string &component_name, int value);
  void set_component_value(const std::string &component_name, float value);
  
  // Text control functions
  void set_component_text(const std::string &component_name, const std::string &text);
  void set_component_text_printf(const std::string &component_name, const char *format, ...);
  
  // Picture control functions
  void set_component_picture(const std::string &component_name, int picture_id);
  void set_component_picture1(const std::string &component_name, int picture_id);
  
  // Color control functions
  void set_component_background_color(const std::string &component_name, int color);
  void set_component_background_color(const std::string &component_name, Color color);
  void set_component_font_color(const std::string &component_name, int color);
  void set_component_font_color(const std::string &component_name, Color color);
  
  // Page control
  void set_page(int page);
  
  // Boot callback registration
  void add_on_boot_callback(std::function<void()> callback) { this->on_boot_callback_ = callback; }
  
  // Current page accessor
  int get_current_page() const { return this->current_page_; }
  
  // Lambda setter for display updates
  void set_lambda(std::function<void(NextionIntelligent &)> lambda) { this->lambda_ = lambda; }

 protected:
  // Send command to Nextion (internal method)
  void send_command(const std::string &command);
  
  // Handle received bytes from Nextion
  void handle_rx_byte_(uint8_t byte);
  
  // State machine for receiving Nextion commands
  enum class NextionRecvState {
    IDLE,    // Waiting for command byte
    COMMAND, // Receiving command data
  };
  
  // State variables for receiving data
  NextionRecvState recv_state_ = NextionRecvState::IDLE;
  uint8_t recv_command_ = 0;
  uint8_t recv_buffer_[4];
  uint8_t recv_buffer_pos_ = 0;
  
  // Callbacks and state
  std::function<void()> on_boot_callback_ = nullptr;
  std::function<void(NextionIntelligent &)> lambda_ = nullptr;
  int current_page_ = 0;
};

} // namespace nextion_intelligent
} // namespace esphome

// Include action classes
#include "nextion_actions.h"
