// SPI LCD panels via esp_lcd. The full framebuffer lives in PSRAM; rows are
// copied through a small DMA-capable bounce buffer on flush.
#include "port.hpp"  // first: pulls in FreeRTOS.h ahead of task.h/queue.h

#include <algorithm>
#include <cstring>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"
#include "panel_box3.hpp"
#include "panel_cores3.hpp"
#include "panel_ws185.hpp"
#include "freertos/task.h"

namespace hgp {
namespace {

const char* TAG = "hg.lcd";
constexpr spi_host_device_t kHost = SPI2_HOST;
constexpr int kBounceRows = 20;
constexpr ledc_channel_t kBlChannel = LEDC_CHANNEL_0;

}  // namespace

bool SpiDisplay::on_trans_done(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void* ctx) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(static_cast<SpiDisplay*>(ctx)->done_, &woken);
  if (woken) portYIELD_FROM_ISR();  // the SPI panel IO ignores this callback's return value
  return false;
}

bool SpiDisplay::begin(const LcdConfig& cfg, i2c_master_bus_handle_t i2c_bus) {
  cfg_ = cfg;
  const bool qspi = cfg.controller == LcdController::St77916;
  bool ili9341 = false;
  bool cores3_e = false;
  if (cfg.controller == LcdController::Box3) {
    if (!i2c_bus) return false;
    if (i2c_master_probe(i2c_bus, 0x24, 50) != ESP_OK) {
      if (i2c_master_probe(i2c_bus, 0x5d, 50) != ESP_OK && i2c_master_probe(i2c_bus, 0x14, 50) != ESP_OK) {
        ESP_LOGE(TAG, "BOX-3 display revision could not be detected");
        return false;
      }
      ili9341 = true;
    }
  }
  controller_name_ = qspi ? "st77916" : ili9341 ? "ili9342" : "st7789";
  if (cfg.controller == LcdController::CoreS3) {
    if (!i2c_bus) return false;
    i2c_device_config_t device = {};
    device.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device.device_address = 0x38;
    device.scl_speed_hz = 100000;
    i2c_master_dev_handle_t touch = nullptr;
    if (i2c_master_bus_add_device(i2c_bus, &device, &touch) != ESP_OK) return false;
    uint8_t version = 0;
    bool detected = false;
    for (int attempt = 0; attempt < 5; ++attempt) {
      const uint8_t work_mode[] = {0x00, 0x00};
      const uint8_t reg = 0xa6;
      if (i2c_master_transmit(touch, work_mode, sizeof(work_mode), 50) == ESP_OK &&
          i2c_master_transmit_receive(touch, &reg, 1, &version, 1, 50) == ESP_OK &&
          (version == 0x10 || version == 0x12)) { detected = true; break; }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    i2c_master_bus_rm_device(touch);
    if (!detected) {
      ESP_LOGE(TAG, "unknown CoreS3 panel revision (touch firmware 0x%02x)", version);
      return false;
    }
    ili9341 = true;
    cores3_e = version == 0x12;
    controller_name_ = cores3_e ? "ili9342e" : "ili9342c";
  }
  const size_t px = static_cast<size_t>(cfg.width) * cfg.height;
  fb_ = static_cast<uint16_t*>(heap_caps_malloc(px * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!fb_) fb_ = static_cast<uint16_t*>(heap_caps_malloc(px * 2, MALLOC_CAP_8BIT));
  bounce_ = static_cast<uint16_t*>(heap_caps_malloc(static_cast<size_t>(cfg.width) * kBounceRows * 2, MALLOC_CAP_DMA));
  if (!fb_ || !bounce_) {
    ESP_LOGE(TAG, "not enough memory for a %ux%u framebuffer", cfg.width, cfg.height);
    return false;
  }
  std::memset(fb_, 0, px * 2);
  done_ = xSemaphoreCreateBinary();
  // A band takes about 2.6 ms on a 320-px panel at 40 MHz. Allow ten times the
  // band at the configured clock, and never less than 100 ms.
  const uint32_t band_ms = static_cast<uint32_t>(cfg.width) * kBounceRows * 16 / (std::max(1, cfg.spi_mhz) * 1000u) + 1;
  bands_.emplace(
      cfg.height, kBounceRows, std::max<uint32_t>(100, 10 * band_ms),
      [this](int y, int rows) {
        const int w = cfg_.width;
        std::memcpy(bounce_, fb_ + static_cast<size_t>(y) * w, static_cast<size_t>(rows) * w * 2);
        return esp_lcd_panel_draw_bitmap(panel_, 0, y, w, y + rows, bounce_) == ESP_OK;
      },
      [this](uint32_t ms) { return xSemaphoreTake(done_, pdMS_TO_TICKS(ms)) == pdTRUE; });

  spi_bus_config_t bus = {};
  bus.mosi_io_num = cfg.mosi;
  bus.miso_io_num = -1;
  bus.sclk_io_num = cfg.sclk;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  if (qspi) {
    bus.data1_io_num = cfg.d1;
    bus.data2_io_num = cfg.d2;
    bus.data3_io_num = cfg.d3;
    bus.flags = SPICOMMON_BUSFLAG_QUAD;
  }
  bus.max_transfer_sz = cfg.width * kBounceRows * 2;
  ESP_ERROR_CHECK(spi_bus_initialize(kHost, &bus, SPI_DMA_CH_AUTO));

  esp_lcd_panel_io_spi_config_t io_cfg = {};
  io_cfg.dc_gpio_num = static_cast<gpio_num_t>(cfg.dc);
  io_cfg.cs_gpio_num = static_cast<gpio_num_t>(cfg.cs);
  io_cfg.pclk_hz = static_cast<uint32_t>(cfg.spi_mhz) * 1000 * 1000;
  io_cfg.lcd_cmd_bits = qspi ? 32 : 8;
  io_cfg.flags.quad_mode = qspi;
  io_cfg.lcd_param_bits = 8;
  io_cfg.spi_mode = 0;
  io_cfg.trans_queue_depth = 4;
  io_cfg.on_color_trans_done = &SpiDisplay::on_trans_done;
  io_cfg.user_ctx = this;
  uint8_t panel_id[4] = {};
  if (qspi) {
    // As in the factory demo, read the panel ID (command 0x04) at 3 MHz. It
    // selects one of the two vendor initialization tables below.
    const uint32_t full_speed = io_cfg.pclk_hz;
    io_cfg.pclk_hz = 3000000;
    if (esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(kHost), &io_cfg, &io_) != ESP_OK) return false;
    const esp_err_t read = esp_lcd_panel_io_rx_param(io_, (0x0Bu << 24) | (0x04u << 8), panel_id, sizeof(panel_id));
    esp_lcd_panel_io_del(io_);
    io_ = nullptr;
    io_cfg.pclk_hz = full_speed;
    ESP_LOGI(TAG, "ST77916 panel ID %02x %02x %02x %02x", panel_id[0], panel_id[1], panel_id[2], panel_id[3]);
    if (read != ESP_OK || panel_id[0] != 0 || panel_id[2] != 0x7f || panel_id[3] != 0x7f ||
        (panel_id[1] != 0x7f && panel_id[1] != 0x02)) {
      ESP_LOGE(TAG, "unknown ST77916 panel revision; refusing guessed initialization");
      return false;
    }
  }
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(kHost), &io_cfg, &io_));

  esp_lcd_panel_dev_config_t panel_cfg = {};
  panel_cfg.reset_gpio_num = static_cast<gpio_num_t>(cfg.rst);
  panel_cfg.rgb_ele_order = cfg.bgr ? LCD_RGB_ELEMENT_ORDER_BGR : LCD_RGB_ELEMENT_ORDER_RGB;
  panel_cfg.bits_per_pixel = 16;
  panel_cfg.flags.reset_active_high = cfg.reset_active_high;
  ili9341_vendor_config_t vendor = {};
  st77916_vendor_config_t st_vendor = {};
  if (qspi) {
    const bool newer = panel_id[1] == 0x02;
    st_vendor.flags.use_qspi_interface = 1;
    st_vendor.init_cmds = newer ? kWs185PanelNew : kWs185PanelDefault;
    st_vendor.init_cmds_size = newer ? sizeof(kWs185PanelNew) / sizeof(kWs185PanelNew[0]) :
                                     sizeof(kWs185PanelDefault) / sizeof(kWs185PanelDefault[0]);
    panel_cfg.vendor_config = &st_vendor;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st77916(io_, &panel_cfg, &panel_));
  } else if (ili9341) {
    if (cfg.controller == LcdController::Box3) {
      vendor.init_cmds = kBox3PanelInit;
      vendor.init_cmds_size = sizeof(kBox3PanelInit) / sizeof(kBox3PanelInit[0]);
      panel_cfg.vendor_config = &vendor;
    } else if (cores3_e) {
      vendor.init_cmds = kCoreS3EPanelInit;
      vendor.init_cmds_size = sizeof(kCoreS3EPanelInit) / sizeof(kCoreS3EPanelInit[0]);
      panel_cfg.vendor_config = &vendor;
    }
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_, &panel_cfg, &panel_));
  } else {
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_, &panel_cfg, &panel_));
  }
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, cfg.invert));
  ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_, cfg.swap_xy));
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_, cfg.mirror_x, cfg.mirror_y));
  ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_, cfg.gap_x, cfg.gap_y));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

  if (cfg.backlight >= 0) {
    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = 5000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t ch = {};
    ch.gpio_num = cfg.backlight;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.channel = kBlChannel;
    ch.timer_sel = LEDC_TIMER_0;
    ch.duty = 0;
    ch.flags.output_invert = cfg.backlight_invert;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    set_backlight(100);
  }
  if (board_backlight) board_backlight(100);
  ESP_LOGI(TAG, "%s %ux%u ready", controller_name_, cfg.width, cfg.height);
  return true;
}

hg::DisplayInfo SpiDisplay::info() const {
  hg::DisplayInfo di;
  di.width = cfg_.width;
  di.height = cfg_.height;
  di.swap_bytes = true;  // the panel wants big-endian RGB565
  di.has_backlight = cfg_.backlight >= 0 || static_cast<bool>(board_backlight);
  di.round = cfg_.round;
  return di;
}

void SpiDisplay::flush(uint16_t y0, uint16_t y1) {
  switch (bands_->flush(y0, y1)) {
    case hg::BandFlush::Event::TimedOut:
      ESP_LOGE(TAG, "LCD transfer timed out; display paused until it completes");
      break;
    case hg::BandFlush::Event::Resumed:
      ESP_LOGW(TAG, "late LCD transfer completed; display resumed");
      break;
    case hg::BandFlush::Event::Failed:
      ESP_LOGE(TAG, "LCD transfer failed; display updates stopped until reboot");
      break;
    case hg::BandFlush::Event::None:
      break;
  }
}

void SpiDisplay::set_backlight(uint8_t percent) {
  if (board_backlight) { board_backlight(percent); return; }
  if (cfg_.backlight < 0) return;
  uint32_t duty = (1023u * std::min<uint8_t>(percent, 100)) / 100u;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, kBlChannel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, kBlChannel);
}

}  // namespace hgp
