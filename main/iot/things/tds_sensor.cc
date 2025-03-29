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

#define TAG "TdsSensor"
#define DEFAULT_VREF 3300        // 默认参考电压3.3V
#define ADC_SAMPLE_COUNT 32      // 采样次数

namespace iot {

class TdsSensor : public Thing {
private:
    adc_channel_t adc_channel_ = ADC_CHANNEL_2;  // TDS模拟输入（GPIO3）
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;  // ADC oneshot句柄
    float voltage_ = 0.0f;
    float tds_value_ = 0.0f;
    float conductivity_ = 0.0f;
    float k_factor_ = 0.67f;
    TaskHandle_t sensor_task_ = nullptr;

    void InitializeTDSChannel() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        adc_handle_ = thing_manager.adc_handle_;

        // 配置通道参数
        adc_oneshot_chan_cfg_t channel_config = {
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_12  // 12位精度
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(
            adc_handle_, adc_channel_, &channel_config));
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

    float ReadVoltage() {
        int adc_reading = 0;
        for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
            int raw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, adc_channel_, &raw));
            adc_reading += raw;
        }
        adc_reading /= ADC_SAMPLE_COUNT;
        
        // 将ADC值转换为电压（V）
        return (float)adc_reading * DEFAULT_VREF / 4095.0f / 1000.0;
    }

    void UpdateTdsValue() {
        voltage_ = ReadVoltage();
        float temperature_ = GetTemperature();
        float compensationCoefficient = 1.0 + 0.02 * (temperature_ - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationVoltage = voltage_ / compensationCoefficient;
        tds_value_ = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5; //convert voltage value to tds value
        conductivity_ = k_factor_ * tds_value_;
        ESP_LOGI(TAG, "K系数: %.2f TDS: %.2f ppm 电压: %.2f V 温度: %.2f °C 电导率: %.2f μS/cm ", 
                k_factor_, tds_value_, voltage_, temperature_, conductivity_);
        
        auto& app = Application::GetInstance();
        if (conductivity_ > 100.0f) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << conductivity_;
            std::string conductivity_str = ss.str();
            std::string states = "{\"name\":\"TdsSensor\",\"state\":{\"conductivity\":\"" + conductivity_str + "\"}}";
            std::string message = "警告！检测到电导率异常（当前电导率为" + conductivity_str + "μS/cm）,请立即检查水缸情况！";
            
            app.Schedule([this, &app, message]() {
                // 播报TDS数据异常
                app.Alert(Lang::Strings::WARNING, message.c_str(), "sad", Lang::Sounds::P3_SUCCESS);
            });
        }
    }

    static void SensorTask(void* arg) {
        TdsSensor* sensor = static_cast<TdsSensor*>(arg);
        while (true) {
            sensor->UpdateTdsValue();
            vTaskDelay(pdMS_TO_TICKS(5000)); // 5秒间隔
        }
    }

public:
    TdsSensor() : Thing("TdsSensor", "水质TDS传感器") {
        InitializeTDSChannel();
        
        // 创建独立任务进行数据采集
        xTaskCreate(SensorTask, "tds_task", 4096, this, 5, &sensor_task_);

        // 定义设备属性
        properties_.AddFloatProperty("conductivity", "当前水电导率，保留两个小数点，单位μS/cm", [this]() -> float {
            return conductivity_;
        });

        properties_.AddFloatProperty("tds", "当前水TDS值，保留两个小数点，单位ppm", [this]() -> float {
            return tds_value_;
        });

        // properties_.AddFloatProperty("voltage", "当前水电压值V", [this]() -> float {
        //     return voltage_;
        // });

        // properties_.AddFloatProperty("k_factor", "当前TDS系数", [this]() -> float { 
        //     return k_factor_; 
        // });

        // methods_.AddMethod("SetterKFactor", "设置TSD系数", ParameterList({
        //     Parameter("k_factor", "0.5到0.8之间的小数", kValueTypeFloat, true)
        // }), [this](const ParameterList& parameters) {
        //     k_factor_ = static_cast<float>(parameters["k_factor"].floating());
        // });

        // 定义手动刷新方法
        methods_.AddMethod("Refresh", "立即刷新TDS数据", ParameterList(), 
            [this](const ParameterList&) {
                UpdateTdsValue();
            });
    }

    ~TdsSensor() {
        if (sensor_task_) vTaskDelete(sensor_task_);
        if (adc_handle_) adc_oneshot_del_unit(adc_handle_);  // 释放ADC资源
    }
};

} // namespace iot

DECLARE_THING(TdsSensor);