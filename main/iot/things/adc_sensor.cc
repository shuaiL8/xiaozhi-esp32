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

#define TAG "AdcSensor"
#define DEFAULT_VREF 3300        // 默认参考电压3.3V
#define ADC_SAMPLE_COUNT 16      // 采样次数

namespace iot {

class AdcSensor : public Thing {
private:
    adc_channel_t ph_channel_ = ADC_CHANNEL_1;  // pH模拟输入（GPIO2）
    adc_channel_t tds_channel_ = ADC_CHANNEL_2;  // TDS模拟输入（GPIO3）
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;  // ADC oneshot句柄
    float tds_voltage_ = 0.0f;
    float tds_value_ = 0.0f;
    float conductivity_ = 0.0f;
    float k_factor_ = 0.67f;
    float ph_value_ = 7.0f;
    float ph_voltage_ = 0.0f;
    float slope = -14.0;    // 斜率 m
    float intercept = 30.24; // 截距 b
    float calibration_offset_ = 0.0f;    // 校准偏移量
    float calibration_slope_ = 0.01786f; // 默认斜率3.3V/14pH=0.2357V/pH
    TaskHandle_t sensor_task_ = nullptr;

    void InitializeChannel() {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));

        // 配置通道参数
        adc_oneshot_chan_cfg_t channel_config_tds = {
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_12  // 12位精度
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(
            adc_handle_, tds_channel_, &channel_config_tds));

        // 配置通道参数
        adc_oneshot_chan_cfg_t channel_config_ph = {
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_12  // 12位精度
        };

        ESP_ERROR_CHECK(adc_oneshot_config_channel(
            adc_handle_, ph_channel_, &channel_config_ph));
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

    float ReadTdsVoltage() {
        int adc_reading = 0;
        for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
            int raw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, tds_channel_, &raw));
            adc_reading += raw;
        }
        adc_reading /= ADC_SAMPLE_COUNT;
        return (float)adc_reading * DEFAULT_VREF / 4095.0f / 1000.0;
    }

    void UpdateTdsValue() {
        tds_voltage_ = ReadTdsVoltage();
        float temperature_ = GetTemperature();
        float compensationCoefficient = 1.0 + 0.02 * (temperature_ - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationVoltage = tds_voltage_ / compensationCoefficient;
        tds_value_ = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5; //convert voltage value to tds value
        conductivity_ = k_factor_ * tds_value_;
        tds_value_ = static_cast<float>(static_cast<int>(tds_value_ * 100 + 0.5)) / 100;
        conductivity_ = static_cast<float>(static_cast<int>(conductivity_ * 100 + 0.5)) / 100;
        ESP_LOGI(TAG, "K系数: %.2f TDS: %.2f ppm 电压: %.2f V 温度: %.2f °C 电导率: %.2f μS/cm ", 
                k_factor_, tds_value_, tds_voltage_, temperature_, conductivity_);
        
        auto& app = Application::GetInstance();
        if (conductivity_ > 100.0f) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << conductivity_;
            std::string conductivity_str = ss.str();
            std::string message = "警告！检测到电导率异常（当前电导率为" + conductivity_str + "μS/cm）,请立即检查水缸情况！";
            
            app.Schedule([this, &app, message]() {
                // 播报TDS数据异常
                app.Alert(Lang::Strings::WARNING, message.c_str(), "sad", Lang::Sounds::P3_SUCCESS);
            });
        }
    }

    float ReadPhVoltage() {
        int adc_reading = 0;
        for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
            int raw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, ph_channel_, &raw));
            adc_reading += raw;
        }
        adc_reading /= ADC_SAMPLE_COUNT;
        return (float)adc_reading * DEFAULT_VREF / 4095.0f / 1000.0;
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
            std::string message = "警告！检测到pH值异常（当前值为" + ph_str + "）！";

            app.Schedule([this, &app, message]() {
                app.Alert(Lang::Strings::WARNING, message.c_str(), "sad", Lang::Sounds::P3_SUCCESS);
            });
        }
    }

    static void SensorTask(void* arg) {
        AdcSensor* sensor = static_cast<AdcSensor*>(arg);
        while (true) {
            sensor->UpdateTdsValue();
            sensor->UpdatePhValue();
            vTaskDelay(pdMS_TO_TICKS(60000)); // 60秒间隔
        }
    }

public:
    AdcSensor() : Thing("AdcSensor", "水质传感器") {
        InitializeChannel();
        
        // 创建独立任务进行数据采集
        xTaskCreate(SensorTask, "tds_task", 4096, this, 1, &sensor_task_);

        // 定义设备属性
        properties_.AddFloatProperty("conductivity", "当前水电导率，单位μS/cm", [this]() -> float {
            return conductivity_;
        });

        properties_.AddFloatProperty("tds", "当前水TDS值，单位ppm", [this]() -> float {
            return tds_value_;
        });
        properties_.AddFloatProperty("ph", "当前pH值（0-14）", [this]() -> float {
            return ph_value_;
        });

        // 定义手动刷新方法
        methods_.AddMethod("RefreshTds", "立即刷新TDS数据", ParameterList(), 
            [this](const ParameterList&) {
                UpdateTdsValue();
        });

        methods_.AddMethod("RefreshPh", "立即刷新pH数据", ParameterList(), 
            [this](const ParameterList&) {
                UpdatePhValue();
        });
    }

    ~AdcSensor() {
        if (sensor_task_) vTaskDelete(sensor_task_);
        if (adc_handle_) adc_oneshot_del_unit(adc_handle_);  // 释放ADC资源
    }
};

} // namespace iot

DECLARE_THING(AdcSensor);