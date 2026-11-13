#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <esp_rom_sys.h>
#include <cstdio>
#include <cmath>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "CompactWifiBoard"


class EnvironmentSensors {
    i2c_master_bus_handle_t bus_;
    i2c_master_dev_handle_t aht_ = nullptr;
    i2c_master_dev_handle_t bmp_ = nullptr;
    struct { uint16_t t1, p1; int16_t t2, t3, p2, p3, p4, p5, p6, p7, p8, p9; } cal_{};
    bool bmp_ready_ = false;
    bool AddDevice(uint8_t address, i2c_master_dev_handle_t* handle) {
        i2c_device_config_t cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = address, .scl_speed_hz = 100 * 1000 };
        return i2c_master_bus_add_device(bus_, &cfg, handle) == ESP_OK;
    }
    bool Write(i2c_master_dev_handle_t dev, const uint8_t* data, size_t len) {
        return dev && i2c_master_transmit(dev, data, len, 100) == ESP_OK;
    }
    bool ReadReg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* data, size_t len) {
        return dev && i2c_master_transmit_receive(dev, &reg, 1, data, len, 100) == ESP_OK;
    }
    bool WriteReg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value) {
        uint8_t b[2] = {reg, value}; return Write(dev, b, sizeof(b));
    }
    bool InitBmp(uint8_t address) {
        if (!AddDevice(address, &bmp_)) return false;
        uint8_t id = 0;
        if (!ReadReg(bmp_, 0xD0, &id, 1) || id != 0x58) {
            i2c_master_bus_rm_device(bmp_); bmp_ = nullptr; return false;
        }
        if (!WriteReg(bmp_, 0xE0, 0xB6)) {
            i2c_master_bus_rm_device(bmp_); bmp_ = nullptr; return false;
        }
        esp_rom_delay_us(10000);
        uint8_t c[24];
        if (!ReadReg(bmp_, 0x88, c, sizeof(c))) {
            i2c_master_bus_rm_device(bmp_); bmp_ = nullptr; return false;
        }
        cal_.t1 = (uint16_t)(c[1] << 8 | c[0]); cal_.t2 = (int16_t)(c[3] << 8 | c[2]);
        cal_.t3 = (int16_t)(c[5] << 8 | c[4]); cal_.p1 = (uint16_t)(c[7] << 8 | c[6]);
        cal_.p2 = (int16_t)(c[9] << 8 | c[8]); cal_.p3 = (int16_t)(c[11] << 8 | c[10]);
        cal_.p4 = (int16_t)(c[13] << 8 | c[12]); cal_.p5 = (int16_t)(c[15] << 8 | c[14]);
        cal_.p6 = (int16_t)(c[17] << 8 | c[16]); cal_.p7 = (int16_t)(c[19] << 8 | c[18]);
        cal_.p8 = (int16_t)(c[21] << 8 | c[20]); cal_.p9 = (int16_t)(c[23] << 8 | c[22]);
        if (cal_.p1 == 0 || !WriteReg(bmp_, 0xF5, 0xA0)) {
            i2c_master_bus_rm_device(bmp_); bmp_ = nullptr; return false;
        }
        // Forced mode: each request starts a fresh temperature/pressure conversion.
        return true;
    }
public:
    explicit EnvironmentSensors(i2c_master_bus_handle_t bus) : bus_(bus) {
        if (AddDevice(0x38, &aht_)) {
            const uint8_t init[] = {0xBE, 0x08, 0x00}; Write(aht_, init, sizeof(init)); esp_rom_delay_us(10000);
        }
        bmp_ready_ = InitBmp(0x76);
        if (!bmp_ready_) bmp_ready_ = InitBmp(0x77);
        ESP_LOGI(TAG, "AHT20: %s, BMP280: %s", aht_ ? "found" : "not found", bmp_ready_ ? "found" : "not found");
    }
    bool ReadAht(float& temperature, float& humidity) {
        if (!aht_) return false;
        const uint8_t cmd[] = {0xAC, 0x33, 0x00};
        if (!Write(aht_, cmd, sizeof(cmd))) return false;
        esp_rom_delay_us(85000);
        uint8_t d[6];
        if (i2c_master_receive(aht_, d, sizeof(d), 100) != ESP_OK || (d[0] & 0x80)) return false;
        uint32_t rh = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
        uint32_t rt = (((uint32_t)d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];
        humidity = rh * 100.0f / 1048576.0f;
        temperature = rt * 200.0f / 1048576.0f - 50.0f;
        return true;
    }
    bool ReadBmp(float& temperature, float& pressure_mmhg) {
        if (!bmp_ready_) return false;
        if (!WriteReg(bmp_, 0xF4, 0x25)) return false; // x1 temperature, x1 pressure, forced mode
        esp_rom_delay_us(10000);
        uint8_t d[6]; if (!ReadReg(bmp_, 0xF7, d, sizeof(d))) return false;
        int32_t ap = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
        int32_t at = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
        double v1 = (((double)at / 16384.0 - cal_.t1 / 1024.0) * cal_.t2);
        double v2 = (pow((double)at / 131072.0 - cal_.t1 / 8192.0, 2) * cal_.t3);
        double tf = v1 + v2; temperature = (float)(tf / 5120.0);
        double p1 = tf / 2.0 - 64000.0;
        double p2 = p1 * p1 * cal_.p6 / 32768.0 + p1 * cal_.p5 * 2.0;
        p2 = p2 / 4.0 + cal_.p4 * 65536.0;
        p1 = (cal_.p3 * p1 * p1 / 524288.0 + cal_.p2 * p1) / 524288.0;
        p1 = (1.0 + p1 / 32768.0) * cal_.p1;
        if (p1 == 0.0) return false;
        double pressure = (1048576.0 - ap - p2 / 4096.0) * 6250.0 / p1;
        p1 = cal_.p9 * pressure * pressure / 2147483648.0;
        p2 = pressure * cal_.p8 / 32768.0;
        pressure += (p1 + p2 + cal_.p7) / 16.0;
        pressure_mmhg = (float)(pressure / 133.322387415);
        return true;
    }
};


class CompactWifiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    EnvironmentSensors* sensors_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSsd1306Display() {
        // SSD1306 config
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif
        ESP_LOGI(TAG, "SSD1306 driver installed");

        // Reset the display
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        // Set the display to on
        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    // 物联网初始化，逐步迁移到 MCP 协议
    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
        sensors_ = new EnvironmentSensors(display_i2c_bus_);
        McpServer::GetInstance().AddTool(
            "environment.get_readings",
            "Получить текущие показания датчиков AHT20 и BMP280: температуру, относительную влажность и атмосферное давление. Вызывай этот инструмент, когда пользователь спрашивает о температуре, влажности или давлении возле устройства. Отвечай по-русски.",
            PropertyList(),
            [this](const PropertyList&) -> ToolResult {
                float ta = 0, rh = 0, tb = 0, p = 0;
                bool a = sensors_ && sensors_->ReadAht(ta, rh);
                bool b = sensors_ && sensors_->ReadBmp(tb, p);
                if (!a && !b) return std::unexpected("Датчики AHT20 и BMP280 не отвечают. Проверьте питание и I2C GPIO 41/42.");
                char result[240] = {};
                if (a && b) snprintf(result, sizeof(result), "AHT20: температура %.1f градуса Цельсия, влажность %.1f процента. BMP280: температура %.1f градуса Цельсия, давление %.1f миллиметра ртутного столба.", ta, rh, tb, p);
                else if (a) snprintf(result, sizeof(result), "AHT20: температура %.1f градуса Цельсия, влажность %.1f процента. BMP280 не отвечает.", ta, rh);
                else snprintf(result, sizeof(result), "BMP280: температура %.1f градуса Цельсия, давление %.1f миллиметра ртутного столба. AHT20 не отвечает.", tb, p);
                // Keep the OLED notification compact; return the full readings for voice.
                char screen_result[96] = {};
                if (a && b) snprintf(screen_result, sizeof(screen_result), "T: %.1fC\nH: %.0f%%\nP: %.0f мм рт. ст.", tb, rh, p);
                else if (a) snprintf(screen_result, sizeof(screen_result), "T: %.1fC\nH: %.0f%%\nBMP280: нет", ta, rh);
                else snprintf(screen_result, sizeof(screen_result), "T: %.1fC\nH: --\nP: %.0f мм рт. ст.", tb, p);
                if (GetDisplay()) GetDisplay()->ShowNotification(screen_result, 6000);
                return std::string(result);
            });
    }

public:
    CompactWifiBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(TOUCH_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        InitializeTools();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(CompactWifiBoard);

