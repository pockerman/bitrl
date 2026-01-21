#include "bitrl/bitrl_consts.h"
#include "bitrl/sensors/sensor_manager.h"

namespace bitrl
{
namespace sensors
{

SensorManager::SensorManager(uint_t n_sensors)
    :
n_sensors_(n_sensors)
{
    if(n_sensors > 0 && n_sensors != consts::INVALID_ID)
    {
       sensors_.reserve(n_sensors);
    }
}

void SensorManager::add(std::shared_ptr<SensorBase> sensor)
{
    sensors_.push_back(sensor);
}

void SensorManager::update()
{
    for (auto& s : sensors_) {
        if (s->is_enabled()) {
            s->read_values();
        }
    }
}

}
}
