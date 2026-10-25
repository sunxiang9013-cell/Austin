#include "wifi_board.h"
#include "display/lcd_display.h"
#include "codecs/box_audio_codec.h"
#include "application.h"
#include "button.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "config.h"
#include "power_save_timer.h"
#include "axp2101.h"
#include "i2c_device.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_timer.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include "settings.h"

#include <esp_lcd_touch_cst816s.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>

#define TAG "WaveshareEsp32s3TouchLCD1inch83"

namespace {

constexpr uint8_t kQmi8658WhoAmIReg = 0x00;
constexpr uint8_t kQmi8658Ctrl1Reg = 0x02;
constexpr uint8_t kQmi8658Ctrl2Reg = 0x03;
constexpr uint8_t kQmi8658Ctrl3Reg = 0x04;
constexpr uint8_t kQmi8658Ctrl7Reg = 0x08;
constexpr uint8_t kQmi8658DataReg = 0x35;
constexpr uint8_t kQmi8658WhoAmIValue = 0x05;

constexpr int kVehicleSampleIntervalMs = 50;
constexpr int64_t kVehicleMotionCooldownUs = 900 * 1000;
constexpr int64_t kDefaultAnimationIntervalUs = 3500 * 1000;
constexpr int64_t kAutoVehicleHoldUs = 8000 * 1000;
constexpr int kTurnGyroRawThreshold = 6000;
constexpr int kLongitudinalAccelDeltaRawThreshold = 1400;
constexpr float kLongitudinalBaselineAlpha = 0.08f;
constexpr int kDefaultAnimationCount = 6;

// Flip these if the board is mounted in the vehicle with the opposite orientation.
constexpr bool kInvertTurnDirection = false;
constexpr bool kInvertLongitudinalDirection = false;
constexpr bool kUseAccelXForLongitudinal = false;

enum class RobotMode {
    kAiCompanion,
    kVehicle,
    kAuto,
};

enum class DisplayOwner {
    kAi,
    kVehicle,
    kSystem,
};

enum class VehicleMotionState {
    kIdle,
    kLeftTurn,
    kRightTurn,
    kAccelerate,
    kBrake,
    kStopped,
};

int16_t MakeInt16(uint8_t lo, uint8_t hi) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

const char* RobotModeText(RobotMode mode) {
    switch (mode) {
        case RobotMode::kAiCompanion:
            return "AI COMPANION";
        case RobotMode::kVehicle:
            return "VEHICLE";
        case RobotMode::kAuto:
            return "AUTO";
        default:
            return "UNKNOWN";
    }
}

RobotMode ParseRobotMode(const std::string& mode) {
    if (mode == "vehicle" || mode == "car" || mode == "drive" || mode == "车载模式") {
        return RobotMode::kVehicle;
    }
    if (mode == "auto" || mode == "自动模式") {
        return RobotMode::kAuto;
    }
    return RobotMode::kAiCompanion;
}

const char* VehicleMotionText(VehicleMotionState state) {
    switch (state) {
        case VehicleMotionState::kLeftTurn:
            return "LEFT";
        case VehicleMotionState::kRightTurn:
            return "RIGHT";
        case VehicleMotionState::kAccelerate:
            return "GO";
        case VehicleMotionState::kBrake:
            return "BRAKE";
        case VehicleMotionState::kStopped:
            return "STOP";
        case VehicleMotionState::kIdle:
            return "IDLE";
        default:
            return "";
    }
}

lv_color_t VehicleMotionColor(VehicleMotionState state) {
    switch (state) {
        case VehicleMotionState::kLeftTurn:
            return lv_color_hex(0x1D4ED8);
        case VehicleMotionState::kRightTurn:
            return lv_color_hex(0x7C3AED);
        case VehicleMotionState::kAccelerate:
            return lv_color_hex(0x16A34A);
        case VehicleMotionState::kBrake:
            return lv_color_hex(0xDC2626);
        case VehicleMotionState::kStopped:
            return lv_color_hex(0xF97316);
        case VehicleMotionState::kIdle:
            return lv_color_hex(0x0891B2);
        default:
            return lv_color_hex(0x111827);
    }
}

class Qmi8658 {
public:
    Qmi8658(i2c_master_bus_handle_t i2c_bus, uint8_t addr) {
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 400 * 1000,
            .scl_wait_us = 0,
            .flags = {
                .disable_ack_check = 0,
            },
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &cfg, &i2c_device_));
    }

    ~Qmi8658() {
        if (i2c_device_ != nullptr) {
            i2c_master_bus_rm_device(i2c_device_);
        }
    }

    bool Initialize() {
        uint8_t who_am_i = 0;
        if (ReadReg(kQmi8658WhoAmIReg, &who_am_i) != ESP_OK) {
            ESP_LOGW(TAG, "QMI8658 WHO_AM_I read failed");
            return false;
        }
        if (who_am_i != kQmi8658WhoAmIValue) {
            ESP_LOGW(TAG, "Unexpected QMI8658 WHO_AM_I: 0x%02x", who_am_i);
            return false;
        }

        // Enable address auto-increment, accelerometer (+/-8g) and gyroscope (+/-512dps).
        return WriteReg(kQmi8658Ctrl1Reg, 0x60) == ESP_OK &&
               WriteReg(kQmi8658Ctrl2Reg, 0x23) == ESP_OK &&
               WriteReg(kQmi8658Ctrl3Reg, 0x53) == ESP_OK &&
               WriteReg(kQmi8658Ctrl7Reg, 0x03) == ESP_OK;
    }

    bool ReadMotion(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
        uint8_t data[12] = {};
        if (ReadRegs(kQmi8658DataReg, data, sizeof(data)) != ESP_OK) {
            return false;
        }

        ax = MakeInt16(data[0], data[1]);
        ay = MakeInt16(data[2], data[3]);
        az = MakeInt16(data[4], data[5]);
        gx = MakeInt16(data[6], data[7]);
        gy = MakeInt16(data[8], data[9]);
        gz = MakeInt16(data[10], data[11]);
        return true;
    }

private:
    i2c_master_dev_handle_t i2c_device_ = nullptr;

    esp_err_t WriteReg(uint8_t reg, uint8_t value) {
        uint8_t payload[2] = {reg, value};
        return i2c_master_transmit(i2c_device_, payload, sizeof(payload), pdMS_TO_TICKS(100));
    }

    esp_err_t ReadReg(uint8_t reg, uint8_t* value) {
        return i2c_master_transmit_receive(i2c_device_, &reg, 1, value, 1, pdMS_TO_TICKS(100));
    }

    esp_err_t ReadRegs(uint8_t reg, uint8_t* data, size_t length) {
        return i2c_master_transmit_receive(i2c_device_, &reg, 1, data, length, pdMS_TO_TICKS(100));
    }
};

class VehicleRobotDisplay : public SpiLcdDisplay {
public:
    VehicleRobotDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                        int width, int height, int offset_x, int offset_y, bool mirror_x,
                        bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y,
                        swap_xy) {}

    void SetupUI() override {
        SpiLcdDisplay::SetupUI();

        DisplayLockGuard lock(this);
        auto screen = lv_screen_active();

        vehicle_layer_ = lv_obj_create(screen);
        lv_obj_set_size(vehicle_layer_, width_, height_);
        lv_obj_align(vehicle_layer_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_border_width(vehicle_layer_, 0, 0);
        lv_obj_set_style_radius(vehicle_layer_, 0, 0);
        lv_obj_set_style_pad_all(vehicle_layer_, 0, 0);
        lv_obj_set_scrollbar_mode(vehicle_layer_, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_flag(vehicle_layer_, LV_OBJ_FLAG_HIDDEN);

        vehicle_card_ = lv_obj_create(vehicle_layer_);
        lv_obj_set_size(vehicle_card_, width_ * 4 / 5, height_ * 2 / 3);
        lv_obj_align(vehicle_card_, LV_ALIGN_CENTER, 0, -8);
        lv_obj_set_style_radius(vehicle_card_, 12, 0);
        lv_obj_set_style_border_width(vehicle_card_, 0, 0);
        lv_obj_set_style_shadow_width(vehicle_card_, 20, 0);
        lv_obj_set_style_shadow_opa(vehicle_card_, LV_OPA_30, 0);
        lv_obj_set_scrollbar_mode(vehicle_card_, LV_SCROLLBAR_MODE_OFF);

        vehicle_label_ = lv_label_create(vehicle_card_);
        lv_obj_set_width(vehicle_label_, width_ * 4 / 5 - 18);
        lv_obj_set_style_text_align(vehicle_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(vehicle_label_, lv_color_white(), 0);
        lv_label_set_long_mode(vehicle_label_, LV_LABEL_LONG_CLIP);
        lv_obj_center(vehicle_label_);

        mode_label_ = lv_label_create(vehicle_layer_);
        lv_obj_set_width(mode_label_, width_ - 12);
        lv_obj_set_style_text_align(mode_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(mode_label_, lv_color_white(), 0);
        lv_label_set_text(mode_label_, "VEHICLE MODE");
        lv_obj_align(mode_label_, LV_ALIGN_BOTTOM_MID, 0, -10);
    }

    void SetRobotMode(RobotMode mode) {
        if (!IsSetupUICalled() || vehicle_layer_ == nullptr || mode_label_ == nullptr) {
            return;
        }

        DisplayLockGuard lock(this);
        active_mode_ = mode;
        display_owner_ = mode == RobotMode::kVehicle ? DisplayOwner::kVehicle : DisplayOwner::kAi;
        lv_label_set_text(mode_label_, RobotModeText(mode));

        if (display_owner_ == DisplayOwner::kAi) {
            lv_obj_add_flag(vehicle_layer_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_remove_flag(vehicle_layer_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(vehicle_layer_);
        }
    }

    void ShowVehicleMotion(VehicleMotionState state) {
        if (!IsSetupUICalled() || vehicle_layer_ == nullptr || vehicle_card_ == nullptr ||
            vehicle_label_ == nullptr) {
            return;
        }

        DisplayLockGuard lock(this);
        display_owner_ = DisplayOwner::kVehicle;
        lv_obj_remove_flag(vehicle_layer_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(vehicle_layer_);
        lv_obj_set_style_bg_color(vehicle_layer_, lv_color_hex(0x020617), 0);
        lv_obj_set_style_bg_opa(vehicle_layer_, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(vehicle_card_, VehicleMotionColor(state), 0);
        lv_label_set_text(vehicle_label_, VehicleMotionText(state));
        PulseVehicleCard(192, 256, 180);
    }

    void ShowDefaultAnimation(int index) {
        if (!IsSetupUICalled() || vehicle_layer_ == nullptr || vehicle_card_ == nullptr ||
            vehicle_label_ == nullptr) {
            return;
        }

        DisplayLockGuard lock(this);
        display_owner_ = DisplayOwner::kVehicle;
        lv_obj_remove_flag(vehicle_layer_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(vehicle_layer_);
        lv_obj_set_style_bg_color(vehicle_layer_, lv_color_hex(0x020617), 0);
        lv_obj_set_style_bg_opa(vehicle_layer_, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(vehicle_card_, lv_color_hex(0x0891B2), 0);
        auto safe_index = std::max(0, index) % kDefaultAnimationCount;
        char text[16] = {};
        snprintf(text, sizeof(text), "IDLE %d", safe_index + 1);
        lv_label_set_text(vehicle_label_, text);
        PulseVehicleCard(232, 256, 420);
    }

    void SetEmotion(const char* emotion) override {
        if (display_owner_ == DisplayOwner::kVehicle) {
            ESP_LOGI(TAG, "AI emotion ignored while vehicle display owns animation: %s", emotion);
            return;
        }
        SpiLcdDisplay::SetEmotion(emotion);
    }

private:
    lv_obj_t* vehicle_layer_ = nullptr;
    lv_obj_t* vehicle_card_ = nullptr;
    lv_obj_t* vehicle_label_ = nullptr;
    lv_obj_t* mode_label_ = nullptr;
    DisplayOwner display_owner_ = DisplayOwner::kAi;
    RobotMode active_mode_ = RobotMode::kAiCompanion;

    void PulseVehicleCard(int32_t from, int32_t to, uint32_t time_ms) {
        lv_obj_set_style_transform_zoom(vehicle_card_, from, 0);
        lv_anim_delete(vehicle_card_, nullptr);
        lv_anim_t zoom_anim;
        lv_anim_init(&zoom_anim);
        lv_anim_set_var(&zoom_anim, vehicle_card_);
        lv_anim_set_values(&zoom_anim, from, to);
        lv_anim_set_time(&zoom_anim, time_ms);
        lv_anim_set_path_cb(&zoom_anim, lv_anim_path_overshoot);
        lv_anim_set_exec_cb(&zoom_anim, [](void* obj, int32_t value) {
            lv_obj_set_style_transform_zoom(static_cast<lv_obj_t*>(obj), value, 0);
        });
        lv_anim_start(&zoom_anim);
    }
};

}  // namespace

class Pmic : public Axp2101 {
public:
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        WriteReg(0x22, 0b110); // PWRON > OFFLEVEL as POWEROFF Source enable
        WriteReg(0x27, 0x10);  // hold 4s to power off

        // Disable All DCs but DC1
        WriteReg(0x80, 0x01);
        // Disable All LDOs
        WriteReg(0x90, 0x00);
        WriteReg(0x91, 0x00);

        // Set DC1 to 3.3V
        WriteReg(0x82, (3300 - 1500) / 100);

        // Set ALDO1 to 3.3V
        WriteReg(0x92, (3300 - 500) / 100);

        // Enable ALDO1(MIC)
        WriteReg(0x90, 0x01);

        WriteReg(0x64, 0x02); // CV charger voltage setting to 4.1V

        WriteReg(0x61, 0x02); // set Main battery precharge current to 50mA
        WriteReg(0x62, 0x08); // set Main battery charger current to 400mA ( 0x08-200mA, 0x09-300mA, 0x0A-400mA )
        WriteReg(0x63, 0x01); // set Main battery term charge current to 25mA
    }
};

class WaveshareEsp32s3TouchLCD1inch83 : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Pmic* pmic_ = nullptr;
    Button boot_button_;
    Display* display_;
    PowerSaveTimer* power_save_timer_;
    TaskHandle_t vehicle_motion_task_ = nullptr;
    std::atomic<int> robot_mode_{static_cast<int>(RobotMode::kAiCompanion)};
    int64_t last_vehicle_motion_time_ = 0;
    int64_t last_default_animation_time_ = 0;
    int64_t last_auto_motion_time_ = 0;
    int default_animation_index_ = 0;
    VehicleMotionState last_motion_state_ = VehicleMotionState::kIdle;

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(20); });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness(); });
        power_save_timer_->OnShutdownRequest([this](){ 
            pmic_->PowerOff(); });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH*  DISPLAY_HEIGHT*  sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
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

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 24 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        esp_lcd_panel_disp_on_off(panel, true);
        display_ = new VehicleRobotDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH - 1,
            .y_max = DISPLAY_HEIGHT - 1,
            .rst_gpio_num = GPIO_NUM_39,
            .int_gpio_num = GPIO_NUM_13,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = DISPLAY_SWAP_XY,
                .mirror_x = DISPLAY_MIRROR_X,
                .mirror_y = DISPLAY_MIRROR_Y,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = {};
        tp_io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_CST816S_ADDRESS;
        tp_io_config.scl_speed_hz = 400 * 1000;
        tp_io_config.control_phase_bytes = 1;
        tp_io_config.dc_bit_offset = 0;
        tp_io_config.lcd_cmd_bits = 8;
        tp_io_config.lcd_param_bits = 0;
        tp_io_config.flags.disable_control_phase = 1;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle));
        ESP_LOGI(TAG, "Initialize touch controller");
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp));
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };
        lvgl_port_add_touch(&touch_cfg);
        ESP_LOGI(TAG, "Touch panel initialized successfully");
    }

    // 初始化工具
    void InitializeTools() {
        auto &mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.system.reconfigure_wifi",
            "End this conversation and enter WiFi configuration mode.\n"
            "**CAUTION** You must ask the user to confirm this action.",
            PropertyList(), [this](const PropertyList& properties) {
                EnterWifiConfigMode();
                return true;
            });

        mcp_server.AddTool("self.robot.set_mode",
            "Switch robot mode. Use this when the user asks to switch to vehicle mode, car mode, AI companion mode, or auto mode. "
            "Valid mode values are: ai, vehicle, auto.",
            PropertyList({
                Property("mode", kPropertyTypeString)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                auto mode_text = properties["mode"].value<std::string>();
                auto mode = ParseRobotMode(mode_text);
                SetRobotMode(mode);
                return std::string("Robot mode switched to ") + RobotModeText(mode);
            });

        mcp_server.AddTool("self.robot.get_mode",
            "Get the current robot mode. Returns AI companion, vehicle, or auto mode.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                return std::string(RobotModeText(GetRobotMode()));
            });
    }

    bool CreateQmi8658(Qmi8658*& imu) {
        constexpr uint8_t candidate_addresses[] = {0x6B, 0x6A};
        for (auto addr : candidate_addresses) {
            if (i2c_master_probe(i2c_bus_, addr, pdMS_TO_TICKS(100)) != ESP_OK) {
                continue;
            }

            auto* candidate = new Qmi8658(i2c_bus_, addr);
            if (candidate->Initialize()) {
                ESP_LOGI(TAG, "QMI8658 initialized at 0x%02x", addr);
                imu = candidate;
                return true;
            }

            delete candidate;
        }

        ESP_LOGW(TAG, "QMI8658 was not found; vehicle motion detection is disabled");
        return false;
    }

    RobotMode GetRobotMode() const {
        return static_cast<RobotMode>(robot_mode_.load());
    }

    void SetRobotMode(RobotMode mode) {
        robot_mode_.store(static_cast<int>(mode));
        last_vehicle_motion_time_ = 0;
        last_default_animation_time_ = 0;
        last_motion_state_ = VehicleMotionState::kIdle;

        auto* vehicle_display = dynamic_cast<VehicleRobotDisplay*>(display_);
        if (vehicle_display != nullptr) {
            vehicle_display->SetRobotMode(mode);
        }

        ESP_LOGI(TAG, "Robot mode: %s", RobotModeText(mode));
    }

    void ShowVehicleMotion(VehicleMotionState state) {
        auto now = esp_timer_get_time();
        if (state == last_motion_state_ && now - last_vehicle_motion_time_ < kVehicleMotionCooldownUs) {
            return;
        }

        auto* vehicle_display = dynamic_cast<VehicleRobotDisplay*>(display_);
        if (vehicle_display == nullptr) {
            return;
        }

        last_vehicle_motion_time_ = now;
        last_motion_state_ = state;
        ESP_LOGI(TAG, "Vehicle motion: %s", VehicleMotionText(state));
        vehicle_display->ShowVehicleMotion(state);
    }

    void ShowDefaultVehicleAnimation(bool force = false) {
        auto now = esp_timer_get_time();
        if (!force && now - last_default_animation_time_ < kDefaultAnimationIntervalUs) {
            return;
        }

        auto* vehicle_display = dynamic_cast<VehicleRobotDisplay*>(display_);
        if (vehicle_display == nullptr) {
            return;
        }

        last_default_animation_time_ = now;
        vehicle_display->ShowDefaultAnimation(default_animation_index_++);
    }

    VehicleMotionState DetectVehicleMotion(int16_t ax, int16_t ay, int16_t gz,
                                           float& longitudinal_baseline,
                                           bool& baseline_initialized) {
        int turn_value = kInvertTurnDirection ? -gz : gz;
        int longitudinal_raw = kUseAccelXForLongitudinal ? ax : ay;
        int longitudinal_value = kInvertLongitudinalDirection ? -longitudinal_raw : longitudinal_raw;

        if (!baseline_initialized) {
            longitudinal_baseline = longitudinal_value;
            baseline_initialized = true;
        } else {
            longitudinal_baseline =
                longitudinal_baseline * (1.0f - kLongitudinalBaselineAlpha) +
                longitudinal_value * kLongitudinalBaselineAlpha;
        }

        auto longitudinal_delta = static_cast<int>(longitudinal_value - longitudinal_baseline);

        if (turn_value > kTurnGyroRawThreshold) {
            return VehicleMotionState::kRightTurn;
        }
        if (turn_value < -kTurnGyroRawThreshold) {
            return VehicleMotionState::kLeftTurn;
        }
        if (longitudinal_delta > kLongitudinalAccelDeltaRawThreshold) {
            return VehicleMotionState::kAccelerate;
        }
        if (longitudinal_delta < -kLongitudinalAccelDeltaRawThreshold) {
            return VehicleMotionState::kBrake;
        }
        return VehicleMotionState::kIdle;
    }

    void VehicleMotionTask() {
        Qmi8658* imu = nullptr;
        bool imu_available = CreateQmi8658(imu);

        int16_t ax = 0;
        int16_t ay = 0;
        int16_t az = 0;
        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;
        float longitudinal_baseline = 0.0f;
        bool baseline_initialized = false;

        while (true) {
            auto mode = GetRobotMode();

            if (mode == RobotMode::kAiCompanion) {
                vTaskDelay(pdMS_TO_TICKS(kVehicleSampleIntervalMs));
                continue;
            }

            VehicleMotionState motion_state = VehicleMotionState::kIdle;
            if (imu_available && imu->ReadMotion(ax, ay, az, gx, gy, gz)) {
                motion_state = DetectVehicleMotion(ax, ay, gz, longitudinal_baseline,
                                                   baseline_initialized);
            }

            if (motion_state != VehicleMotionState::kIdle) {
                last_auto_motion_time_ = esp_timer_get_time();
                ShowVehicleMotion(motion_state);
            } else if (mode == RobotMode::kVehicle) {
                ShowDefaultVehicleAnimation();
            } else if (mode == RobotMode::kAuto) {
                auto now = esp_timer_get_time();
                if (last_auto_motion_time_ != 0 && now - last_auto_motion_time_ < kAutoVehicleHoldUs) {
                    ShowDefaultVehicleAnimation();
                } else {
                    auto* vehicle_display = dynamic_cast<VehicleRobotDisplay*>(display_);
                    if (vehicle_display != nullptr) {
                        vehicle_display->SetRobotMode(RobotMode::kAuto);
                    }
                }
            }

            vTaskDelay(pdMS_TO_TICKS(kVehicleSampleIntervalMs));
        }
    }

    void InitializeVehicleMotion() {
        xTaskCreate(
            [](void* arg) {
                auto* board = static_cast<WaveshareEsp32s3TouchLCD1inch83*>(arg);
                board->VehicleMotionTask();
            },
            "vehicle_motion",
            5120,
            this,
            3,
            &vehicle_motion_task_);
    }

public:
    WaveshareEsp32s3TouchLCD1inch83() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializePowerSaveTimer();
        InitializeCodecI2c();
        InitializeAxp2101();
        InitializeSpi();
        InitializeDisplay();
        InitializeTouch();
        InitializeButtons();
        InitializeTools();
        SetRobotMode(RobotMode::kAiCompanion);
        InitializeVehicleMotion();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7210_ADDR, 
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual bool GetBatteryLevel(int &level, bool &charging, bool &discharging) override {
        static bool last_discharging = false;
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        if (discharging != last_discharging)
        {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }

        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(WaveshareEsp32s3TouchLCD1inch83);
