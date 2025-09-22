#include "nextion_simple.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"

#if defined(USE_ESP_IDF)
#include <esp_http_client.h>
#include <esp_heap_caps.h>
#endif

#include <cinttypes>

namespace esphome {
namespace nextion_simple {

static const char *const TAG = "nextion_simple.upload";

// ========= Common (Arduino placeholder) =========
bool NextionSimple::upload_tft_arduino_() {
  ESP_LOGE(TAG, "Arduino framework upload not implemented.");
  this->upload_in_progress_ = false;
  return false;
}

// ========= ESP-IDF implementation =========
#if defined(USE_ESP_IDF)

static constexpr size_t NX_STREAM_CHUNK = 1024;

bool NextionSimple::wait_for_ack_idf_(uint32_t timeout_ms, std::string &out) {
  out.clear();
  const uint32_t deadline = millis() + timeout_ms;
  uint32_t last_data = millis();
  while (millis() < deadline) {
    while (this->uart_parent_->available()) {
      uint8_t b;
      if (!this->uart_parent_->read_byte(&b)) break;
      out.push_back(static_cast<char>(b));
      last_data = millis();
    }
    // malé timeout okna, ať se to nezasekne když už něco přišlo
    if (!out.empty() && (millis() - last_data) > 10) break;
    delay(2);
    App.feed_wdt();
  }
  return !out.empty();
}

bool NextionSimple::prepare_nextion_for_upload_idf_(uint32_t baud_rate) {
  // Display nesmí spát
  this->send_command_printf("sleep=0");
  this->send_command_printf("dim=100");
  delay(250);

  // Vyčisti RX
  while (this->uart_parent_->available()) {
    uint8_t d; if (!this->uart_parent_->read_byte(&d)) break;
  }

  // whmi-wris <length>,<baud>,1
  char cmd[64];
  int n = snprintf(cmd, sizeof(cmd), "whmi-wris %" PRIu32 ",%" PRIu32 ",1", this->content_length_, baud_rate);
  if (n <= 0 || (size_t) n >= sizeof(cmd)) {
    ESP_LOGE(TAG, "Failed to format whmi-wris");
    return false;
  }
  this->send_command(cmd, (size_t) n);

  // Přepnout UART baud (ESP strana) pokud je třeba
  if (baud_rate != this->original_baud_rate_) {
    ESP_LOGD(TAG, "Changing baud rate from %" PRIu32 " to %" PRIu32, this->original_baud_rate_, baud_rate);
    this->uart_parent_->set_baud_rate(baud_rate);
    this->uart_parent_->load_settings();
  }

  // Čekej na 0x05 ("ready")
  std::string resp;
  if (!this->wait_for_ack_idf_(5000, resp)) {
    ESP_LOGE(TAG, "Timeout waiting upload ACK");
    return false;
  }

  bool ok = resp.find(static_cast<char>(0x05)) != std::string::npos;
  ESP_LOGD(TAG, "Upload prep resp [%s] len=%u",
           format_hex_pretty(reinterpret_cast<const uint8_t*>(resp.data()), resp.size()).c_str(),
           (unsigned)resp.size());
  return ok;
}

// Range upload (HEAD → GET s Range) – převod z originálu na naši třídu
int NextionSimple::upload_by_chunks_idf_(void *http_client_v, uint32_t &range_start) {
  auto http_client = reinterpret_cast<esp_http_client_handle_t>(http_client_v);

  uint32_t range_size = this->tft_size_ - range_start;
  uint32_t range_end = ((upload_first_chunk_sent_ || this->tft_size_ < 4096) ? this->tft_size_ : 4096) - 1;

  if (range_size == 0 || range_end <= range_start) {
    ESP_LOGE(TAG, "Invalid range start=%" PRIu32 " end=%" PRIu32, range_start, range_end);
    return -1;
  }

  char range_header[32];
  snprintf(range_header, sizeof(range_header), "bytes=%" PRIu32 "-%" PRIu32, range_start, range_end);
  esp_http_client_set_header(http_client, "Range", range_header);

  esp_err_t err = esp_http_client_open(http_client, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));
    return -1;
  }

  const int chunk_size = esp_http_client_fetch_headers(http_client);
  if (chunk_size <= 0) {
    ESP_LOGE(TAG, "Failed to get chunk content length: %d", chunk_size);
    esp_http_client_close(http_client);
    return -1;
  }

  // Alokace 4k (interní/PSRAM podle konfigurace)
  uint8_t *buffer = (uint8_t*) heap_caps_malloc(4096, MALLOC_CAP_DEFAULT);
  if (!buffer) {
    ESP_LOGE(TAG, "Failed to allocate upload buffer");
    esp_http_client_close(http_client);
    return -1;
  }

  while (true) {
    App.feed_wdt();
    const uint16_t want = this->content_length_ < 4096 ? this->content_length_ : 4096;
    uint16_t read_len = 0;
    uint8_t retries = 0;

    while (retries < 5 && read_len < want) {
      int r = esp_http_client_read(http_client, reinterpret_cast<char *>(buffer) + read_len, want - read_len);
      if (r > 0) { read_len += (uint16_t) r; retries = 0; }
      else { retries++; vTaskDelay(pdMS_TO_TICKS(2)); }
      App.feed_wdt();
    }

    if (read_len != want) {
      ESP_LOGE(TAG, "Short read: %u of %u", (unsigned)read_len, (unsigned)want);
      free(buffer);
      esp_http_client_close(http_client);
      return -1;
    }

    // Stream do Nextionu (raw)
    size_t sent = 0;
    while (sent < read_len) {
      size_t n = read_len - sent;
      if (n > NX_STREAM_CHUNK) n = NX_STREAM_CHUNK;
      this->uart_parent_->write_array(buffer + sent, n);
      sent += n;
      // yield
#if defined(USE_ESP_IDF)
      vTaskDelay(pdMS_TO_TICKS(0));
#endif
    }

    // Po každém bloku čekáme na odpověď (0x05 OK nebo 0x08 partial)
    std::string ack;
    this->wait_for_ack_idf_(upload_first_chunk_sent_ ? 500 : 5000, ack);

    this->content_length_ -= read_len;
    const float pct = 100.0f * (this->tft_size_ - this->content_length_) / this->tft_size_;
#ifdef USE_PSRAM
    ESP_LOGD(TAG,
             "Uploaded %0.2f%%, remaining %" PRIu32 " B, free: %" PRIu32 " (DRAM) + %" PRIu32 " (PSRAM)",
             pct, this->content_length_,
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#else
    ESP_LOGD(TAG, "Uploaded %0.2f%%, remaining %" PRIu32 " B, free: %" PRIu32,
             pct, this->content_length_, (uint32_t)esp_get_free_heap_size());
#endif
    upload_first_chunk_sent_ = true;

    if (!ack.empty()) {
      const uint8_t *ab = reinterpret_cast<const uint8_t*>(ack.data());
      // 0x08 + 4B offset → požadavek na partial resume
      if (ab[0] == 0x08 && ack.size() >= 5) {
        uint32_t result = 0;
        for (int j = 0; j < 4; ++j) result += (uint32_t)ab[j + 1] << (8 * j);
        if (result > 0) {
          ESP_LOGI(TAG, "Nextion requested resume at %" PRIu32, result);
          this->content_length_ = this->tft_size_ - result;
          range_start = result;
        } else {
          range_start = range_end + 1;
        }
        free(buffer);
        esp_http_client_close(http_client);
        return range_end + 1;
      } else if (ab[0] != 0x05 && ab[0] != 0x08) {
        ESP_LOGE(TAG, "Invalid ACK: [%s]",
                 format_hex_pretty(reinterpret_cast<const uint8_t*>(ack.data()), ack.size()).c_str());
        free(buffer);
        esp_http_client_close(http_client);
        return -1;
      }
    }

    if (read_len == 0) break;  // konec
  }

  range_start = range_end + 1;
  free(buffer);
  esp_http_client_close(http_client);
  return range_end + 1;
}

bool NextionSimple::upload_tft_esp_idf_() {
  ESP_LOGD(TAG, "Nextion TFT upload requested");
  ESP_LOGD(TAG, "URL: %s", this->tft_url_.c_str());

  if (this->is_updating_) {
    ESP_LOGW(TAG, "Currently uploading");
    this->upload_in_progress_ = false;
    return false;
  }
  if (!network::is_connected()) {
    ESP_LOGE(TAG, "Network is not connected");
    this->upload_in_progress_ = false;
    return false;
  }

  this->is_updating_ = true;

  // Zapamatuj původní baud
  this->original_baud_rate_ = this->uart_parent_->get_baud_rate();

  // HEAD pro velikost
  esp_http_client_config_t cfg = {};
  cfg.url = this->tft_url_.c_str();
  cfg.cert_pem = nullptr;
  cfg.method = HTTP_METHOD_HEAD;
  cfg.timeout_ms = 15000;
  cfg.disable_auto_redirect = false;
  cfg.max_redirection_count = 10;

  auto http = esp_http_client_init(&cfg);
  if (!http) {
    ESP_LOGE(TAG, "esp_http_client_init failed");
    this->is_updating_ = false; this->upload_in_progress_ = false;
    return false;
  }
  esp_http_client_set_header(http, "Connection", "keep-alive");

  if (esp_http_client_perform(http) != ESP_OK) {
    ESP_LOGE(TAG, "HTTP HEAD perform failed");
    esp_http_client_cleanup(http);
    this->is_updating_ = false; this->upload_in_progress_ = false;
    return false;
  }
  int status = esp_http_client_get_status_code(http);
  if (status != 200 && status != 206) {
    ESP_LOGE(TAG, "Unexpected HTTP status: %d", status);
    esp_http_client_cleanup(http);
    this->is_updating_ = false; this->upload_in_progress_ = false;
    return false;
  }

  this->tft_size_ = esp_http_client_get_content_length(http);
  ESP_LOGD(TAG, "TFT file size: %" PRIu32 " B", this->tft_size_);
  if (this->tft_size_ < 4096 || this->tft_size_ > 134217728) {
    ESP_LOGE(TAG, "File size out of range");
    esp_http_client_cleanup(http);
    this->is_updating_ = false; this->upload_in_progress_ = false;
    return false;
  }
  this->content_length_ = this->tft_size_;

  // Připrav Nextion k uploadu
  // Volíme baud: pokud není v podpoře, nech původní
  static const uint32_t SUPPORTED[] = {2400,4800,9600,19200,31250,38400,57600,115200,230400,250000,256000,512000,921600};
  uint32_t desired_baud = 921600;
  bool ok_baud = false;
  for (auto b : SUPPORTED) if (b == desired_baud) { ok_baud = true; break; }
  if (!ok_baud) desired_baud = this->original_baud_rate_;

  if (!this->prepare_nextion_for_upload_idf_(desired_baud)) {
    ESP_LOGE(TAG, "Nextion not ready for upload");
    esp_http_client_cleanup(http);
    this->is_updating_ = false; this->upload_in_progress_ = false;
    return false;
  }

  // Přepnout klienta na GET
  if (esp_http_client_set_method(http, HTTP_METHOD_GET) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set GET");
    esp_http_client_cleanup(http);
    this->is_updating_ = false; this->upload_in_progress_ = false;
    return false;
  }

  ESP_LOGD(TAG, "Uploading TFT to Nextion...");
  uint32_t position = 0;
  this->upload_first_chunk_sent_ = false;

  while (this->content_length_ > 0) {
    int r = this->upload_by_chunks_idf_(http, position);
    if (r < 0) {
      ESP_LOGE(TAG, "Upload failed");
      esp_http_client_close(http);
      esp_http_client_cleanup(http);
      // restore baud
      uint32_t cur = this->uart_parent_->get_baud_rate();
      if (cur != this->original_baud_rate_) {
        this->uart_parent_->set_baud_rate(this->original_baud_rate_);
        this->uart_parent_->load_settings();
      }
      // cleanup flags
      this->is_updating_ = false;
      this->upload_in_progress_ = false;
      // runtime zpět do write-only
      this->send_command_printf("bkcmd=0"); this->bkcmd_ = 0;
      this->enter_writeonly_mode_();
      return false;
    }
    App.feed_wdt();
  }

  ESP_LOGI(TAG, "TFT upload successful");

  esp_http_client_close(http);
  esp_http_client_cleanup(http);

  // Obnov baud
  uint32_t cur = this->uart_parent_->get_baud_rate();
  if (cur != this->original_baud_rate_) {
    ESP_LOGD(TAG, "Restoring baud %" PRIu32 " -> %" PRIu32, cur, this->original_baud_rate_);
    this->uart_parent_->set_baud_rate(this->original_baud_rate_);
    this->uart_parent_->load_settings();
  }

  // Vypni odpovědi a vrať write-only
  this->send_command_printf("bkcmd=0"); this->bkcmd_ = 0;
  this->enter_writeonly_mode_();

  this->is_updating_ = false;
  this->upload_in_progress_ = false;

  // (volitelné) restart ESP po úspěchu – některé buildy to dělají:
  delay(1500);
  arch_restart();

  return true;
}

#endif // USE_ESP_IDF

} // namespace nextion_simple
} // namespace esphome
