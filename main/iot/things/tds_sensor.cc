#include "iot/thing.h"
#include "board.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_adc/adc_oneshot.h"

#define TAG "TdsSensor"
#define DEFAULT_VREF 3300        // 默认参考电压3.3V
#define ADC_SAMPLE_COUNT 32      // 采样次数

namespace iot {

class TdsSensor : public Thing {
private:
    adc_channel_t adc_channel_ = ADC_CHANNEL_2;  // 默认使用ADC1通道2（GPIO3）
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;  // ADC oneshot句柄
    float voltage_ = 0.0f;
    float tds_value_ = 0.0f;
    float k_factor_ = 0.67f;
    TaskHandle_t sensor_task_ = nullptr;

    void InitializeADC() {
        // 配置ADC oneshot
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,  // 对应ADC1
            .ulp_mode = ADC_ULP_MODE_DISABLE
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));

        // 配置通道参数
        adc_oneshot_chan_cfg_t channel_config = {
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_12  // 12位精度
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(
            adc_handle_, adc_channel_, &channel_config));
    }

    float ReadVoltage() {
        int adc_reading = 0;
        for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
            int raw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle_, adc_channel_, &raw));
            adc_reading += raw;
        }
        adc_reading /= ADC_SAMPLE_COUNT;
        
        // 将ADC值转换为电压（mV）
        return (float)adc_reading * DEFAULT_VREF / 4095.0f;
    }

    void UpdateTdsValue() {
        voltage_ = ReadVoltage();
        // 简化TDS计算公式：TDS = k * voltage
        tds_value_ = k_factor_ * voltage_;
        ESP_LOGI(TAG, "K系数: %.2f TDS: %.2f ppm, 电压: %.2f mV", 
                k_factor_, tds_value_, voltage_);
    }

    static void SensorTask(void* arg) {
        TdsSensor* sensor = static_cast<TdsSensor*>(arg);
        while (true) {
            sensor->UpdateTdsValue();
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

public:
    TdsSensor() : Thing("TdsSensor", "水质TDS传感器") {
        InitializeADC();
        
        // 创建独立任务进行数据采集
        xTaskCreate(SensorTask, "tds_task", 4096, this, 5, &sensor_task_);

        // 定义设备属性
        properties_.AddFloatProperty("tds", "当前水TDS值ppm", [this]() -> float {
            return tds_value_;
        });

        properties_.AddFloatProperty("voltage", "当前水电压值mV", [this]() -> float {
            return voltage_;
        });

        properties_.AddFloatProperty("k_factor", "当前TDS系数", [this]() -> float { 
            return k_factor_; 
        });

        methods_.AddMethod("SetterKFactor", "设置TSD系数", ParameterList({
            Parameter("k_factor", "0.5到0.8之间的小数", kValueTypeFloat, true)
        }), [this](const ParameterList& parameters) {
            k_factor_ = static_cast<float>(parameters["k_factor"].floating());
        });

        // 定义手动刷新方法
        methods_.AddMethod("Refresh", "立即刷新数据", ParameterList(), 
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