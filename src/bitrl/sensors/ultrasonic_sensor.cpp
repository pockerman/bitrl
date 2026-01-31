#include "bitrl/bitrl_config.h"
#include "bitrl/sensors/ultrasonic_sensor.h"

#ifdef BITRL_LOG
#include <boost/log/trivial.hpp>
#endif

#include <algorithm>

namespace bitrl
{
namespace sensors
{
const  std::string UltrasonicSensor::SENSOR_TYPE = "UltrasonicSensor";


void UltrasonicSensor::init()
{
    auto n_values = backend_ -> n_read_values();
    this -> values_.resize(n_values, -1.0);
    this -> enable();
}

const std::vector<real_t>& UltrasonicSensor::read_values()
{
    if (!this -> is_enabled())
    {
#ifdef BITRL_LOG
        BOOST_LOG_TRIVIAL(warning)<<"Sensor: " << this -> sensor_name() << " is not enabled. Returning dummy values";
#endif
        this -> values_;
    }

    // TODO: Make sure that len(read_values) == len(values)
    auto read_values = backend_ -> read_values();

    std::copy(read_values.begin(), read_values.end(), values_.begin());
    return this -> values_;
}

}
}
