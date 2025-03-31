#include "iot/thing.h"
#include "board.h"
#include "assets/lang_config.h"
#include "application.h"

#include <esp_log.h>

#define TAG "Timer"

namespace iot {

    // 这里仅定义 Timer 的属性和方法，不包含具体的实现
    class Timer : public Thing {
    private:
        bool toggle_timer_ = false;
        time_t timer_end_time_;
        TaskHandle_t timer_task_ = nullptr;

        void addTimer(int time_range) {
            ESP_LOGI(TAG, "计时时长: %d", time_range);
            timer_end_time_ = time(NULL) + time_range;
            toggle_timer_ = true;
            char time_str[64];
            strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&timer_end_time_));
            std::string message = "计时设置成功：" + std::string(time_str);
            notify(message);
        }

        void notify(std::string message) {
            auto &app = Application::GetInstance();
            app.Schedule([this, &app, message]() {
                app.Alert(Lang::Strings::INFO, message.c_str(), "happy", Lang::Sounds::P3_SUCCESS);
            });
        }

        static void TimerTask(void *arg) {
            Timer* timer = static_cast<Timer*>(arg);
            while (true) {
                // Set status to clock "YYYY-MM-DD HH:MM:SS"
                time_t now = time(NULL);
                char time_str[64];
                strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));
                ESP_LOGI(TAG, "当前时间: %s", time_str);

                if (timer->toggle_timer_ && timer->timer_end_time_ <= now) {
                    timer->notify("计时完成!");
                    timer->toggle_timer_ = false;
                }
                vTaskDelay(pdMS_TO_TICKS(1000)); // 1秒间隔
            }
        }

    public:
        Timer() : Thing("Timer", "计时器") {
        
            // 创建独立任务进行计时
            xTaskCreate(TimerTask, "timer_task", 4096, this, 5, &timer_task_);

            // 定义设备的属性
            properties_.AddStringProperty("curTimer", "当前计时器状态", [this]() -> std::string {
            if (toggle_timer_) {
                char time_str[64];
                strftime(time_str, sizeof(time_str), "%Y-%M-%D %H:%M:%S", localtime(&timer_end_time_));   
                return time_str;
            }
            return "当前无计时器"; });

            methods_.AddMethod("addTimer", "新增计时器", ParameterList(
                {Parameter("time_range", "0到60之间的整数", kValueTypeNumber, true)
                }), [this](const ParameterList &parameters) {
                    addTimer(static_cast<int>(parameters["time_range"].number())); 
                });
        }

        ~Timer() {
            if (timer_task_) {
                vTaskDelete(timer_task_);
            }
        }
    };

} // namespace iot

DECLARE_THING(Timer);