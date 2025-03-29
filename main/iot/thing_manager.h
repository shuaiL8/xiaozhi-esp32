#ifndef THING_MANAGER_H
#define THING_MANAGER_H


#include "thing.h"

#include <cJSON.h>

#include <vector>
#include <memory>
#include <functional>
#include <map>
#include "esp_adc/adc_oneshot.h"

namespace iot {

class ThingManager {
public:
    static ThingManager& GetInstance() {
        static ThingManager instance;
        return instance;
    }
    ThingManager(const ThingManager&) = delete;
    ThingManager& operator=(const ThingManager&) = delete;

    void AddThing(Thing* thing);

    void InitializeADC();

    std::string GetDescriptorsJson();
    bool GetStatesJson(std::string& json, bool delta = false);
    void Invoke(const cJSON* command);
    std::string GetThingStateJson(std::string name);
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;

private:
    ThingManager() = default;
    ~ThingManager() = default;

    std::vector<Thing*> things_;
    std::map<std::string, std::string> last_states_;
};


} // namespace iot

#endif // THING_MANAGER_H
