#include "iot/thing.h"
#include "board.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "TemperatureSensor"

namespace iot {

class TemperatureSensor : public Thing {
private:
#ifdef CONFIG_IDF_TARGET_ESP32
    gpio_num_t gpio_num_ = GPIO_NUM_9;
#else
    gpio_num_t gpio_num_ = GPIO_NUM_10;
#endif
    float temperature_ = 0.0f;
    TaskHandle_t sensor_task_ = nullptr;

    void InitializeGpio() {
        gpio_config_t config = {
            .pin_bit_mask = (1ULL << gpio_num_),
            .mode = GPIO_MODE_INPUT_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&config));
    }

    void OneWireReset() {
        gpio_set_direction(gpio_num_, GPIO_MODE_OUTPUT_OD);
        gpio_set_level(gpio_num_, 0);
        esp_rom_delay_us(480);
        gpio_set_level(gpio_num_, 1);
        gpio_set_direction(gpio_num_, GPIO_MODE_INPUT);
        esp_rom_delay_us(70);
        esp_rom_delay_us(410);
    }

    void WriteBit(bool bit) {
        gpio_set_direction(gpio_num_, GPIO_MODE_OUTPUT_OD);
        gpio_set_level(gpio_num_, 0);
        esp_rom_delay_us(bit ? 1 : 60);
        gpio_set_level(gpio_num_, 1);
        esp_rom_delay_us(bit ? 60 : 1);
    }

    void WriteByte(uint8_t byte) {
        for (int i = 0; i < 8; i++) {
            WriteBit(byte & 0x01);
            byte >>= 1;
        }
    }

    uint8_t ReadBit() {
        gpio_set_direction(gpio_num_, GPIO_MODE_OUTPUT_OD);
        gpio_set_level(gpio_num_, 0);
        esp_rom_delay_us(1);
        gpio_set_direction(gpio_num_, GPIO_MODE_INPUT);
        esp_rom_delay_us(14);
        uint8_t bit = gpio_get_level(gpio_num_);
        esp_rom_delay_us(45);
        return bit;
    }

    uint8_t ReadByte() {
        uint8_t byte = 0;
        for (int i = 0; i < 8; i++) {
            byte |= (ReadBit() << i);
        }
        return byte;
    }

    float ReadTemperature() {
        OneWireReset();
        WriteByte(0xCC); // Skip ROM
        WriteByte(0x44); // Convert T
        esp_rom_delay_us(750000);

        OneWireReset();
        WriteByte(0xCC); // Skip ROM
        WriteByte(0xBE); // Read Scratchpad

        uint8_t data[9];
        for (int i = 0; i < 9; i++) {
            data[i] = ReadByte();
        }

        int16_t raw = (data[1] << 8) | data[0];
        return raw / 16.0f;
    }

    static void SensorTask(void* arg) {
        TemperatureSensor* sensor = reinterpret_cast<TemperatureSensor*>(arg);
        while (true) {
            float temp = sensor->ReadTemperature();
            sensor->temperature_ = temp;
            ESP_LOGI(TAG, "当前温度: %.2f°C", temp);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

public:
    TemperatureSensor() : Thing("TemperatureSensor", "温度传感器") {
        InitializeGpio();
        
        // 创建独立任务进行温度采集
        xTaskCreate(SensorTask, "temp_task", 4096, this, 5, &sensor_task_);

        // 定义设备属性
        properties_.AddFloatProperty("temperature", "当前水温值°C", [this]() -> float {
            return temperature_;
        });

        // 定义手动刷新方法（可选）
        methods_.AddMethod("Refresh", "立即刷新温度", ParameterList(), 
            [this](const ParameterList&) {
                temperature_ = ReadTemperature();
            });
    }

    ~TemperatureSensor() {
        if (sensor_task_) {
            vTaskDelete(sensor_task_);
        }
    }
};

} // namespace iot

DECLARE_THING(TemperatureSensor);