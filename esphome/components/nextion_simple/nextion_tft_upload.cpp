#include "nextion_simple.h"
#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include "esphome/core/util.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"
#include <memory>

namespace esphome {
namespace nextion_simple {

static const char *const TAG = "nextion_simple.upload";

// Common function to prepare Nextion for upload
bool NextionSimple::prepare_nextion_for_upload_() {
  ESP_LOGD(TAG, "Setting bkcmd=3 for upload");
  this->send_command("bkcmd=3");
  
  // Give time for the command to be processed
  #ifdef USE_ESP_IDF
  vTaskDelay(pdMS_TO_TICKS(50));
  #else
  delay(50);
  #endif
  
  // Clear receive buffer
  while (this->uart_parent_->available()) {
    uint8_t byte;
    this->uart_parent_->read_byte(&byte);
  }
  
  // Prepare command to tell Nextion about the TFT file
  char command[128];
  uint32_t baud_rate = this->uart_parent_->get_baud_rate();
  
  // Using command format according to Nextion documentation
  sprintf(command, "whmi-wris %" PRIu32 ",%" PRIu32 ",1", this->content_length_, baud_rate);
  
  // Send upload command to Nextion
  ESP_LOGV(TAG, "Sending upload command: %s", command);
  this->uart_parent_->write_array((const uint8_t *)command, strlen(command));
  this->uart_parent_->write_array((const uint8_t *)"\xFF\xFF\xFF", 3);
  this->uart_parent_->flush();
  
  // Wait for nextion to respond with 0x05
  unsigned long timeout = millis() + 10000;
  int response = -1;
  
  while (millis() < timeout) {
    if (this->uart_parent_->available() > 0) {
      uint8_t byte;
      if (this->uart_parent_->read_byte(&byte)) {
        response = byte;
        ESP_LOGV(TAG, "Received byte: 0x%02X", byte);
        if (response == 0x05) {
          ESP_LOGD(TAG, "Nextion ready to receive data");
          return true;
        }
      }
    }
    
    #ifdef USE_ESP_IDF
    vTaskDelay(pdMS_TO_TICKS(10));
    #else
    delay(10);
    #endif
  }
  
  ESP_LOGE(TAG, "Nextion did not acknowledge upload command (received: 0x%02X)", response);
  return false;
}

// Common function to wait for ACK from Nextion
bool NextionSimple::wait_for_nextion_ack_() {
  unsigned long timeout = millis() + 10000;
  bool ack_received = false;
  
  while (millis() < timeout && !ack_received) {
    if (this->uart_parent_->available() > 0) {
      uint8_t byte;
      if (this->uart_parent_->read_byte(&byte) && byte == 0x05) {
        ack_received = true;
        ESP_LOGV(TAG, "Received ACK from Nextion for chunk");
        break;
      }
    }
    
    #ifdef USE_ESP_IDF
    vTaskDelay(pdMS_TO_TICKS(1));
    #else
    delay(1);
    #endif
  }
  
  if (!ack_received) {
    ESP_LOGE(TAG, "No ACK received from Nextion after sending chunk");
    return false;
  }
  
  return true;
}

// Common function to send data to Nextion
bool NextionSimple::send_data_to_nextion_(const uint8_t* data, size_t data_size) {
  size_t written = 0;
  size_t to_write = data_size;
  
  // Send data in smaller chunks to avoid UART buffer overflow
  while (written < to_write) {
    // Get optimal write chunk size based on available buffer space
    size_t write_chunk = std::min(to_write - written, (size_t)64);
    
    // Write chunk to UART
    this->uart_parent_->write_array(data + written, write_chunk);
    this->uart_parent_->flush();
    written += write_chunk;
    
    // Give some time for the Nextion to process the data
    if (written % 512 == 0) {
      #ifdef USE_ESP_IDF
      vTaskDelay(pdMS_TO_TICKS(5));
      #else
      delay(5);
      #endif
    }
  }
  
  return true;
}

// Reset Nextion after upload (common for both frameworks)
bool NextionSimple::upload_end_(bool successful) {
  ESP_LOGD(TAG, "Nextion TFT upload finished: %s", successful ? "success" : "failed");
  this->upload_in_progress_ = false;
  
  if (successful) {
    ESP_LOGI(TAG, "Resetting Nextion...");
    this->send_command("rest");
    
    #ifdef USE_ESP_IDF
    vTaskDelay(pdMS_TO_TICKS(1000));
    #else
    delay(1000);
    #endif
    
    ESP_LOGI(TAG, "Nextion reset completed");
  }
  
  return successful;
}

// Get platform-specific free heap
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

#ifdef USE_ARDUINO
#include <HTTPClient.h>

// Upload TFT file in chunks
int NextionSimple::upload_by_chunks_(HTTPClient &http_client, uint32_t &range_start) {
  const uint32_t UPLOAD_CHUNK_SIZE = 4096; // Standard Nextion chunk size
  
  // Adjust chunk size based on available memory
  uint32_t free_heap = this->get_free_heap_();
  uint32_t chunk_size = std::min(UPLOAD_CHUNK_SIZE, free_heap / 4);
  
  if (chunk_size < 256) {
    ESP_LOGE(TAG, "Not enough free heap memory for upload: %u bytes", free_heap);
    return -1;
  }
  
  // Calculate the chunk end position
  uint32_t range_end = range_start + chunk_size - 1;
  if (range_end >= this->content_length_ - 1) {
    range_end = this->content_length_ - 1;
  }
  
  // Set range header for HTTP request
  char range_header[64];
  sprintf(range_header, "bytes=%u-%u", range_start, range_end);
  http_client.addHeader("Range", range_header);
  
  // Get the chunk from the HTTP server
  int http_code = http_client.GET();
  if (http_code != 206) {
    ESP_LOGE(TAG, "HTTP GET failed, code: %d", http_code);
    return -1;
  }
  
  // Get the content from the HTTP response
  WiFiClient *stream = http_client.getStreamPtr();
  size_t available_size = stream->available();
  
  if (available_size == 0) {
    ESP_LOGE(TAG, "No data received");
    return -1;
  }
  
  // Create buffer for the data
  std::unique_ptr<uint8_t[]> buf(new uint8_t[available_size]);
  if (buf == nullptr) {
    ESP_LOGE(TAG, "Memory allocation failed");
    return -1;
  }
  
  // Read data into buffer
  size_t read_size = stream->readBytes(buf.get(), available_size);
  if (read_size != available_size) {
    ESP_LOGE(TAG, "Read size doesn't match available size");
    return -1;
  }
  
  // Send data to Nextion using common method
  if (!this->send_data_to_nextion_(buf.get(), read_size)) {
    return -1;
  }
  
  // Update range and content length
  range_start = range_end + 1;
  
  // Wait for ACK from Nextion if not the last chunk
  if (this->content_length_ > 0 && !this->wait_for_nextion_ack_()) {
    return -1;
  }
  
  return read_size;
}

// Upload TFT file to Nextion using Arduino framework
void NextionSimple::upload_tft_arduino_() {
  HTTPClient http_client;
  ESP_LOGD(TAG, "Creating HTTP connection");
  http_client.setTimeout(10000); // 10 seconds timeout
  http_client.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
  // Get the content length first with a HEAD request
  ESP_LOGV(TAG, "Getting content length");
  http_client.begin(this->tft_url_);
  http_client.addHeader("User-Agent", "ESPHome");
  int http_code = http_client.sendRequest("HEAD");
  
  if (http_code != 200) {
    ESP_LOGE(TAG, "Failed to get content length, HTTP code: %d", http_code);
    http_client.end();
    this->upload_end_(false);
    return;
  }
  
  // Read content length from headers
  this->content_length_ = http_client.getSize();
  http_client.end();
  
  // Check if file size is valid
  if (this->content_length_ < 4096) {
    ESP_LOGE(TAG, "File size check failed: %d bytes is too small", this->content_length_);
    this->upload_end_(false);
    return;
  }
  
  // Feed watchdog
  App.feed_wdt();
  
  
  this->send_command("DRAKJHS256");
  #ifdef USE_ESP_IDF
  vTaskDelay(pdMS_TO_TICKS(100));
  #else
  delay(100);
  #endif

  // Prepare Nextion for upload using common method
  if (!this->prepare_nextion_for_upload_()) {
    this->upload_end_(false);
    return;
  }
  
  // Begin HTTP transfer
  ESP_LOGD(TAG, "Uploading TFT to Nextion:");
  ESP_LOGD(TAG, " URL: %s", this->tft_url_.c_str());
  ESP_LOGD(TAG, " File size: %u bytes", this->content_length_);
  http_client.begin(this->tft_url_);
  http_client.addHeader("User-Agent", "ESPHome");
  
  // Start upload process
  uint32_t position = 0;
  uint32_t remaining_length = this->content_length_;
  while (remaining_length > 0) {
    App.feed_wdt();
    
    ESP_LOGV(TAG, "Uploading chunk: %u bytes remaining", this->content_length_);
    
    int retries = 0;
    int upload_result = -1;
    while (upload_result < 0 && retries < 3) {
      upload_result = this->upload_by_chunks_(http_client, position);
      if (upload_result < 0) {
        ESP_LOGW(TAG, "Retrying chunk upload (%d/3)...", retries + 1);
      }
      retries++;
    }
    
    if (upload_result < 0) {
      ESP_LOGE(TAG, "Error uploading TFT to Nextion!");
      http_client.end();
      this->upload_end_(false);
      return;
    }
    
    App.feed_wdt();
    ESP_LOGV(TAG, "Free heap: %" PRIu32 ", Bytes left: %" PRIu32, this->get_free_heap_(), this->content_length_);
  }
  
  ESP_LOGD(TAG, "Successfully uploaded TFT to Nextion!");
  http_client.end();
  
  // Reset Nextion using common method
  this->upload_end_(true);
}
#endif // USE_ARDUINO

#ifdef USE_ESP_IDF
#include "esp_http_client.h"
#include "esp_heap_caps.h"

// Upload TFT file in chunks for ESP-IDF
int NextionSimple::upload_by_chunks_(esp_http_client_handle_t http_client, uint32_t &range_start) {
  const uint32_t UPLOAD_CHUNK_SIZE = 4096; // Standard Nextion chunk size
  
  // Adjust chunk size based on available memory
  uint32_t free_heap = this->get_free_heap_();
  uint32_t chunk_size = std::min(UPLOAD_CHUNK_SIZE, free_heap / 4);
  
  if (chunk_size < 256) {
    ESP_LOGE(TAG, "Not enough free heap memory for upload: %u bytes", free_heap);
    return -1;
  }
  
  // Calculate the chunk end position
  uint32_t range_end = range_start + chunk_size - 1;
  if (range_end >= this->content_length_ - 1) {
    range_end = this->content_length_ - 1;
  }
  
  // Set range header for HTTP request
  char range_header[64];
  sprintf(range_header, "bytes=%u-%u", range_start, range_end);
  esp_http_client_set_header(http_client, "Range", range_header);

  // Perform HTTP request
  esp_err_t err = esp_http_client_perform(http_client);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
    return -1;
  }
  
  int status_code = esp_http_client_get_status_code(http_client);
  if (status_code != 206) {
    ESP_LOGE(TAG, "HTTP GET failed, code: %d", status_code);
    return -1;
  }
  
  // Get content length from response
  int content_length = esp_http_client_get_content_length(http_client);
  if (content_length <= 0) {
    ESP_LOGE(TAG, "No data received");
    return -1;
  }
  
  // Allocate buffer for data with proper memory typing
  std::unique_ptr<uint8_t[]> buf(new uint8_t[content_length]);
  if (buf == nullptr) {
    ESP_LOGE(TAG, "Memory allocation failed");
    return -1;
  }
  
  // Read data from HTTP response
  int read_size = 0;
  int read_len = 0;
  while (read_size < content_length) {
    read_len = esp_http_client_read(http_client, (char *)(buf.get() + read_size), content_length - read_size);
    if (read_len <= 0) {
      break;
    }
    read_size += read_len;
  }
  
  if (read_size != content_length) {
    ESP_LOGE(TAG, "Read size doesn't match content length: %d vs %d", read_size, content_length);
    return -1;
  }
  
  // Send data to Nextion using common method
  if (!this->send_data_to_nextion_(buf.get(), read_size)) {
    return -1;
  }
  
  // Update range and content length
  range_start = range_end + 1;
  //this->on_upload_progress_.call(position, this->content_length_);
  
  // Wait for ACK from Nextion if not the last chunk
  if (this->content_length_ > 0 && !this->wait_for_nextion_ack_()) {
    return -1;
  }
  
  return read_size;
}

// Upload TFT file to Nextion using ESP-IDF framework
void NextionSimple::upload_tft_esp_idf_() {
  ESP_LOGD(TAG, "Creating HTTP connection");
  
  // Initialize ESP HTTP client with improved configuration
  esp_http_client_config_t config = {};
  config.url = this->tft_url_.c_str();
  config.timeout_ms = 10000;
  config.buffer_size = 512;
  config.user_agent = "ESPHome";
  config.skip_cert_common_name_check = true; // For HTTPS
  
  esp_http_client_handle_t http_client = esp_http_client_init(&config);
  if (http_client == nullptr) {
    ESP_LOGE(TAG, "Failed to initialize HTTP client");
    this->upload_end_(false);
    return;
  }
  
  // Get content length first
  ESP_LOGV(TAG, "Getting content length");
  esp_err_t set_method_result = esp_http_client_set_method(http_client, HTTP_METHOD_HEAD);
  if (set_method_result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set HTTP method to HEAD: %s", esp_err_to_name(set_method_result));
    esp_http_client_cleanup(http_client);
    this->upload_end_(false);
    return;
  }
  
  esp_err_t err = esp_http_client_perform(http_client);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "HTTP HEAD request failed: %s", esp_err_to_name(err));
    ESP_LOGE(TAG, "URL: %s", this->tft_url_.c_str());
    esp_http_client_cleanup(http_client);
    this->upload_end_(false);
    return;
  }
  
  // Get content length
  this->content_length_ = esp_http_client_get_content_length(http_client);
  if (this->content_length_ < 4096) {
    ESP_LOGE(TAG, "File size check failed: %d bytes is too small", this->content_length_);
    esp_http_client_cleanup(http_client);
    this->upload_end_(false);
    return;
  }
  
  // Set method to GET for data transfer
  set_method_result = esp_http_client_set_method(http_client, HTTP_METHOD_GET);
  if (set_method_result != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set HTTP method to GET: %s", esp_err_to_name(set_method_result));
    esp_http_client_cleanup(http_client);
    this->upload_end_(false);
    return;
  }
  
  // Feed watchdog
  App.feed_wdt();
  
  
  this->send_command("DRAKJHS256");
  #ifdef USE_ESP_IDF
  vTaskDelay(pdMS_TO_TICKS(100));
  #else
  delay(100);
  #endif

  // Prepare Nextion for upload using common method
  if (!this->prepare_nextion_for_upload_()) {
    esp_http_client_cleanup(http_client);
    this->upload_end_(false);
    return;
  }
  
  // Begin HTTP transfer
  ESP_LOGD(TAG, "Uploading TFT to Nextion:");
  ESP_LOGD(TAG, " URL: %s", this->tft_url_.c_str());
  ESP_LOGD(TAG, " File size: %u bytes", this->content_length_);
  
  // Start upload process
  uint32_t position = 0;
  uint32_t remaining_length = this->content_length_;
  while (remaining_length > 0) {
    App.feed_wdt();
    
    ESP_LOGV(TAG, "Uploading chunk: %u bytes remaining", this->content_length_);
    
    int retries = 0;
    int upload_result = -1;
    while (upload_result < 0 && retries < 3) {
      upload_result = this->upload_by_chunks_(http_client, position);
      if (upload_result < 0) {
        ESP_LOGW(TAG, "Retrying chunk upload (%d/3)...", retries + 1);
      }
      retries++;
    }

    if (upload_result < 0) {
      ESP_LOGE(TAG, "Error uploading TFT to Nextion!");
      esp_http_client_cleanup(http_client);
      this->upload_end_(false);
      return;
    }
    
    if (upload_result < 0) {
      ESP_LOGE(TAG, "Error uploading TFT to Nextion!");
      esp_http_client_cleanup(http_client);
      this->upload_end_(false);
      return;
    }
    
    App.feed_wdt();
    ESP_LOGV(TAG, "Free heap: %" PRIu32 ", Bytes left: %" PRIu32, this->get_free_heap_(), this->content_length_);
  }
  
  ESP_LOGD(TAG, "Successfully uploaded TFT to Nextion!");
  esp_http_client_cleanup(http_client);
  
  // Reset Nextion using common method
  this->upload_end_(true);
}
#endif // USE_ESP_IDF

// Main entry point for TFT upload
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

} // namespace nextion_simple
} // namespace esphome
