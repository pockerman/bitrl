#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "bitrl/bitrl_types.h"
#include "bitrl/sensors/sensor_base.h"
#include "bitrl/sensors/backends/range_sensor_backend_base.h"

#include <memory>
#include <vector>

namespace bitrl
{
namespace sensors
{
/**
 * @class UltrasonicSensor
 * @ingroup bitrl_sensors
 * @brief Class for modelling ultrasonic sensors
 */
class UltrasonicSensor final: public SensorBase
{
public:

    /**
     * @brief The type of the sensor
     */
    static const  std::string SENSOR_TYPE;

    /**
     * @brief Constructor
     * @param backend
     */
    explicit UltrasonicSensor(std::shared_ptr<bitrl::sensors::backends::RangeSensorBackendBase> backend,
        const std::string name = bitrl::consts::INVALID_STR);

    /**
     * @brief Initialize the sensor. Set the is is_enabled_ flag to true.
     * and performs any other initializations required by the sensor
     */
    virtual void init();

    /**
     * @brief Reads the sensor values and updates the values held internally
     * @return An instance of str::vector<real_t>
     */
    virtual const std::vector<real_t>& read_values();

    /**
     * @return An instance of std::string of the backend type. If the implementation
     * does not use a specific backend returns bitrl::consts::INVALID_STR
     */
    virtual std::string backend_type_str()const;

    /**
     * @return An instance of std::string indicating the units the sensor is using.
     * If units have not been established this returns bitrl::consts::INVALID_STR
     */
    virtual std::string sensor_units()const;

private:
    /**
     * @brief The backend used for this sensor
     */
    std::shared_ptr<bitrl::sensors::backends::RangeSensorBackendBase> backend_;

};

inline UltrasonicSensor::UltrasonicSensor(std::shared_ptr<bitrl::sensors::backends::RangeSensorBackendBase> backend,
                                         const std::string name)
    :
SensorBase(UltrasonicSensor::SENSOR_TYPE, name),
backend_(backend)
{}

inline
std::string UltrasonicSensor::backend_type_str()const{return backend_ -> backend_type_str();}

inline
std::string UltrasonicSensor::sensor_units()const{return backend_ -> sensor_units();}


}
}

#endif //ULTRASONIC_SENSOR_H
