#include "iot/thing.h"
#include "iot/thing_manager.h"
#include "board.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_adc/adc_oneshot.h"
#include <cJSON.h>
#include "application.h"
#include "assets/lang_config.h"
#include <cstring>
#include <cstdio>

#define TAG "PhSensor"
#define DEFAULT_VREF 3300        // 默认参考电压3.3V
#define ADC_SAMPLE_COUNT 16      // 采样次数

namespace iot {

class PhSensor : public Thing {
private:
    adc_channel_t ph_channel_ = ADC_CHANNEL_1;    // pH模拟输入（GPIO2）
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;  // ADC oneshot句柄
    float ph_value_ = 7.0f;
    float ph_voltage_ = 0.0f;
    float slope = -14.0;    // 斜率 m
    float intercept = 30.24; // 截距 b
    float calibration_offset_ = 0.0f;    // 校准偏移量
    float calibration_slope_ = 0.01786f; // 默认斜率3.3V/14pH=0.2357V/pH
    TaskHandle_t sensor_task_ = nullptr;

    void InitializePHChannel() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        adc_handle_ = thing_manager.adc_handle_;

        // 配置通道参数
        adc_oneshot_chan_cfg_t channel_config = {
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_12  // 12位精度
        };

        ESP_ERROR_CHECK(adc_oneshot_config_channel(
            adc_handle_, ph_channel_, &channel_config));
    }

    float GetTemperature() {
        float temperature = 25.0f; // 默认值
        auto& thing_manager = iot::ThingManager::GetInstance();
        std::string stateStr = thing_manager.GetThingStateJson("TemperatureSensor"); // {"name":"TemperatureSensor","state":{"temperature":19.375000}}

        cJSON* root = cJSON_Parse(stateStr.c_str());
        if (root == nullptr) {
            ESP_LOGE(TAG, "Failed to parse JSON");
            return temperature;
        }

        // 获取 "state" 对象
        cJSON* state_obj = cJSON_GetObjectItem(root, "state");
        if (!cJSON_IsObject(state_obj)) {
            ESP_LOGE(TAG, "Invalid JSON structure: state not found");
            cJSON_Delete(root);
            return temperature;
        }

        // 获取 "temperature" 值
        cJSON* temperature_item = cJSON_GetObjectItem(state_obj, "temperature");
        if (!cJSON_IsNumber(temperature_item)) {
            ESP_LOGE(TAG, "Invalid JSON structure: temperature not found or not a number");
            cJSON_Delete(root);
            return temperature;
        }
        temperature = static_cast<float>(temperature_item->valuedouble);
        cJSON_Delete(root);
        return temperature;
    }

    float ReadPhVoltage() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        SemaphoreHandle_t adc_mutex = thing_manager.adc_mutex;
        if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int adc_reading = 0;
            for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
                int raw = 0;
                ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, ph_channel_, &raw));
                adc_reading += raw;
            }
            adc_reading /= ADC_SAMPLE_COUNT;
            xSemaphoreGive(adc_mutex);  // 释放锁
            return (float)adc_reading * DEFAULT_VREF / 4095.0f / 1000.0;
        } else {
            // 处理超时错误
            return -1.0f;
        }
    }

    void UpdatePhValue() {
        ph_voltage_ = ReadPhVoltage(); 
        float temperature_ = GetTemperature();

        // 基础pH计算（根据传感器特性调整公式）
        // float raw_ph = (ph_voltage_ - 1.45) / 0.17 + 7.0; // 示例公式
        float raw_ph = slope * ph_voltage_ + intercept; 
        
        // 温度补偿（示例补偿系数0.03pH/°C）
        ph_value_ = raw_ph + (25.0f - temperature_) * 0.03;
        ph_value_ = static_cast<float>(static_cast<int>(ph_value_ * 100 + 0.5)) / 100;

        ESP_LOGI(TAG, "电压: %.2fV | 温度: %.2f°C | pH值: %.2f", 
                ph_voltage_, temperature_, ph_value_);

        auto& app = Application::GetInstance();
        if (ph_value_ < 4.0f || ph_value_ > 10.0f) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << ph_value_;
            std::string ph_str = ss.str();
            std::string states = "{\"name\":\"PhSensor\",\"state\":{\"ph\":\"" + ph_str + "\"}}";
            std::string message = "警告！检测到pH值异常（当前值为" + ph_str + "）！";

            app.Schedule([this, &app, message]() {
                app.Alert(Lang::Strings::WARNING, message.c_str(), "sad", Lang::Sounds::P3_SUCCESS);
            });
        }
    }

    static void SensorTask(void* arg) {
        PhSensor* sensor = static_cast<PhSensor*>(arg);
        while (true) {
            sensor->UpdatePhValue();
            vTaskDelay(pdMS_TO_TICKS(60000)); // 60秒间隔
        }
    }

public:
    PhSensor() : Thing("PhSensor", "水质pH传感器") {
        InitializePHChannel();

        xTaskCreate(SensorTask, "ph_task", 4096, this, 1, &sensor_task_);

        // 设备属性定义
        properties_.AddFloatProperty("ph", "当前pH值（0-14）", [this]() -> float {
            return ph_value_;
        });

        // 定义手动刷新方法
        methods_.AddMethod("Refresh", "立即刷新pH数据", ParameterList(), 
            [this](const ParameterList&) {
                UpdatePhValue();
        });
    }

    ~PhSensor() {
        if (sensor_task_) vTaskDelete(sensor_task_);
        if (adc_handle_) adc_oneshot_del_unit(adc_handle_);  // 释放ADC资源
    }
};

} // namespace iot

DECLARE_THING(PhSensor);