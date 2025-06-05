#include "nextion_simple.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"
#include <memory>

namespace esphome {
namespace nextion_simple {

static const char *const TAG = "nextion_simple.upload";

bool NextionSimple::prepare_nextion_for_upload_() {
  ESP_LOGD(TAG, "Setting bkcmd=3 for upload");
  this->send_command_cstr("bkcmd=3");
#ifdef USE_ESP_IDF
  vTaskDelay(pdMS_TO_TICKS(50));
#else
  delay(50);
#endif
  while (this->uart_parent_->available()) {
    uint8_t dummy;
    this->uart_parent_->read_byte(&dummy);
  }
  char cmd[64];
  uint32_t baud = this->uart_parent_->get_baud_rate();
  int n = snprintf(cmd, sizeof(cmd), "whmi-wris %u,%u,1", this->content_length_, baud);
  if (n < 0 || static_cast<size_t>(n) >= sizeof(cmd)) {
    ESP_LOGE(TAG, "Error formatting whmi-wris command");
    return false;
  }
  ESP_LOGV(TAG, "Sending upload command: %s", cmd);
  this->uart_parent_->write_array(reinterpret_cast<const uint8_t *>(cmd), static_cast<size_t>(n));
  const uint8_t term[3] = {0xFF, 0xFF, 0xFF};
  this->uart_parent_->write_array(term, 3);
  this->uart_parent_->flush();
  uint32_t timeout = millis() + 10000;
  while (millis() < timeout) {
    if (this->uart_parent_->available()) {
      uint8_t b;
      if (this->uart_parent_->read_byte(&b) && b == 0x05) {
        ESP_LOGD(TAG, "Nextion ready for data");
        return true;
      }
    }
#ifdef USE_ESP_IDF
    vTaskDelay(pdMS_TO_TICKS(10));
#else
    delay(10);
#endif
  }
  ESP_LOGE(TAG, "Nextion did not ACK upload command (timeout)");
  return false;
}

bool NextionSimple::wait_for_nextion_ack_() {
  uint32_t timeout = millis() + 10000;
  while (millis() < timeout) {
    if (this->uart_parent_->available()) {
      uint8_t b;
      if (this->uart_parent_->read_byte(&b) && b == 0x05) {
        ESP_LOGV(TAG, "ACK received from Nextion");
        return true;
      }
    }
#ifdef USE_ESP_IDF
    vTaskDelay(pdMS_TO_TICKS(1));
#else
    delay(1);
#endif
  }
  ESP_LOGE(TAG, "Timeout: no ACK from Nextion");
  return false;
}

bool NextionSimple::send_data_to_nextion_(const uint8_t* data, size_t data_size) {
  size_t sent = 0;
  while (sent < data_size) {
    size_t chunk = (data_size - sent < 64) ? (data_size - sent) : 64;
    this->uart_parent_->write_array(data + sent, chunk);
    this->uart_parent_->flush();
    sent += chunk;
    if (sent % 512 == 0) {
#ifdef USE_ESP_IDF
      vTaskDelay(pdMS_TO_TICKS(5));
#else
      delay(5);
#endif
    }
  }
  return true;
}

bool NextionSimple::upload_end_(bool successful) {
  ESP_LOGD(TAG, "Upload TFT completed: %s", successful ? "OK" : "FAIL");
  this->upload_in_progress_ = false;
  if (successful) {
    ESP_LOGI(TAG, "Resetting Nextion after upload");
    this->send_command_cstr("rest");
#ifdef USE_ESP_IDF
    vTaskDelay(pdMS_TO_TICKS(1000));
#else
    delay(1000);
#endif
    ESP_LOGI(TAG, "Nextion reset complete");
  }
  return successful;
}

#ifdef USE_ARDUINO
#include <HTTPClient.h>

bool NextionSimple::upload_tft_arduino_() {
  HTTPClient http;
  http.setTimeout(10000);
  http.begin(this->tft_url_);
  http.addHeader("User-Agent", "ESPHome");
  int code = http.sendRequest("HEAD");
  if (code != 200) {
    ESP_LOGE(TAG, "HEAD request failed: %d", code);
    http.end();
    return false;
  }
  this->content_length_ = http.getSize();
  http.end();
  if (this->content_length_ < 4096) {
    ESP_LOGE(TAG, "File too small: %u bytes", this->content_length_);
    return false;
  }
  App.feed_wdt();
  this->send_command_cstr("DRAKJHS256");
#ifdef USE_ESP_IDF
  vTaskDelay(pdMS_TO_TICKS(100));
#else
  delay(100);
#endif
  if (!this->prepare_nextion_for_upload_()) {
    return false;
  }
  http.begin(this->tft_url_);
  http.addHeader("User-Agent", "ESPHome");
  uint32_t position = 0;
  while (position < this->content_length_) {
    App.feed_wdt();
    int retries = 0;
    bool chunk_ok = false;
    while (retries < 3 && !chunk_ok) {
      uint32_t freeh = this->get_free_heap_();
      uint32_t chunk_size = std::min<uint32_t>(4096, freeh / 4);
      uint32_t range_end = std::min<uint32_t>(position + chunk_size - 1, this->content_length_ - 1);
      char range_header[64];
      snprintf(range_header, sizeof(range_header), "bytes=%u-%u", position, range_end);
      http.addHeader("Range", range_header);
      int code2 = http.GET();
      if (code2 != 206) {
        ESP_LOGW(TAG, "HTTP GET chunk failed: %d", code2);
        http.end();
        return false;
      }
      WiFiClient *stream = http.getStreamPtr();
      size_t avail = stream->available();
      if (avail == 0) {
        ESP_LOGE(TAG, "No data in chunk");
        http.end();
        return false;
      }
      std::unique_ptr<uint8_t[]> buf(new uint8_t[avail]);
      size_t read_size = stream->readBytes(buf.get(), avail);
      if (read_size != avail) {
        ESP_LOGE(TAG, "Read %u of %u bytes", read_size, avail);
        http.end();
        return false;
      }
      if (!this->send_data_to_nextion_(buf.get(), read_size)) {
        http.end();
        return false;
      }
      position = range_end + 1;
      if (!this->wait_for_nextion_ack_()) {
        ESP_LOGW(TAG, "ACK error, retry %d", retries + 1);
        retries++;
      } else {
        chunk_ok = true;
      }
    }
    if (!chunk_ok) {
      ESP_LOGE(TAG, "Failed to send chunk after 3 retries");
      http.end();
      return false;
    }
  }
  ESP_LOGD(TAG, "Upload TFT completed: %u bytes", this->content_length_);
  http.end();
  return true;
}
#endif  // USE_ARDUINO

#ifdef USE_ESP_IDF
#include "esp_http_client.h"
#include "esp_heap_caps.h"

bool NextionSimple::upload_tft_esp_idf_() {
  esp_http_client_config_t cfg = {};
  cfg.url = this->tft_url_.c_str();
  cfg.timeout_ms = 10000;
  cfg.buffer_size = 512;
  cfg.user_agent = "ESPHome";
  cfg.skip_cert_common_name_check = true;
  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (!client) {
    ESP_LOGE(TAG, "Failed to init HTTP client");
    return false;
  }
  esp_http_client_set_method(client, HTTP_METHOD_HEAD);
  if (esp_http_client_perform(client) != ESP_OK) {
    ESP_LOGE(TAG, "HEAD request failed: %s", esp_err_to_name(esp_http_client_get_errno(client)));
    esp_http_client_cleanup(client);
    return false;
  }
  this->content_length_ = esp_http_client_get_content_length(client);
  if (this->content_length_ < 4096) {
    ESP_LOGE(TAG, "File too small: %u bytes", this->content_length_);
    esp_http_client_cleanup(client);
    return false;
  }
  esp_http_client_set_method(client, HTTP_METHOD_GET);
  App.feed_wdt();
  this->send_command_cstr("DRAKJHS256");
#ifdef USE_ESP_IDF
  vTaskDelay(pdMS_TO_TICKS(100));
#else
  delay(100);
#endif
  if (!this->prepare_nextion_for_upload_()) {
    esp_http_client_cleanup(client);
    return false;
  }
  uint32_t pos = 0;
  while (pos < this->content_length_) {
    App.feed_wdt();
    int retries = 0;
    bool chunk_ok = false;
    while (retries < 3 && !chunk_ok) {
      uint32_t freeh = this->get_free_heap_();
      uint32_t chunk_size = std::min<uint32_t>(4096, freeh / 4);
      uint32_t end = std::min<uint32_t>(pos + chunk_size - 1, this->content_length_ - 1);
      char range_header[64];
      snprintf(range_header, sizeof(range_header), "bytes=%u-%u", pos, end);
      esp_http_client_set_header(client, "Range", range_header);
      if (esp_http_client_perform(client) != ESP_OK) {
        ESP_LOGW(TAG, "Chunk HTTP failed: %s", esp_err_to_name(esp_http_client_get_errno(client)));
        retries++;
        continue;
      }
      int status = esp_http_client_get_status_code(client);
      if (status != 206) {
        ESP_LOGW(TAG, "Expected 206, got %d", status);
        retries++;
        continue;
      }
      int len = esp_http_client_get_content_length(client);
      if (len <= 0) {
        ESP_LOGE(TAG, "No data in chunk");
        retries++;
        continue;
      }
      auto buf = std::make_unique<uint8_t[]>(len);
      int total_read = 0;
      while (total_read < len) {
        int r = esp_http_client_read(client, reinterpret_cast<char *>(buf.get() + total_read), len - total_read);
        if (r <= 0) break;
        total_read += r;
      }
      if (total_read != len) {
        ESP_LOGE(TAG, "Read %d of %d bytes", total_read, len);
        retries++;
        continue;
      }
      if (!this->send_data_to_nextion_(buf.get(), len)) {
        esp_http_client_cleanup(client);
        return false;
      }
      pos = end + 1;
      if (!this->wait_for_nextion_ack_()) {
        ESP_LOGW(TAG, "ACK failed, retry %d", retries + 1);
        retries++;
      } else {
        chunk_ok = true;
      }
    }
    if (!chunk_ok) {
      ESP_LOGE(TAG, "Failed to send chunk after 3 retries");
      esp_http_client_cleanup(client);
      return false;
    }
  }
  ESP_LOGD(TAG, "Upload TFT complete: %u bytes", this->content_length_);
  esp_http_client_cleanup(client);
  return true;
}
#endif  // USE_ESP_IDF

}  // namespace nextion_simple
}  // namespace esphome
