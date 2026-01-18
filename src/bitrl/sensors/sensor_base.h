#ifndef SENSOR_BASE_H
#define SENSOR_BASE_H


#include "../bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"
#include <vector>
#include <string>
namespace bitrl
{
namespace sensors
{
/**
 * @class SensorBase
 * @ingroup bitrl_sensors
 * @brief Class for modelling sensors. The interface follows closely
 * the interface exposed by Webots see: https://cyberbotics.com/doc/guide/sensors
 * In this token a Sensor is a Device and has:
 * - sampling period
 * - enabled/disabled
 * - Last value
 * - Sensor units
 * Implementations can utilize noise and resolution.
 *
 */
class SensorBase
{
public:
    /**
     * @brief Destructor
     */
    virtual ~SensorBase()=default;

    /**
     * @brief Initialize the sensor. Set the is is_enabled_ flag to true.
     * and performs any other initializations required by the sensor
     */
    virtual void init()=0;

    /**
     * @return Reads the sensor values and updates the values held internally
     */
    virtual const std::vector<real_t>& read_values()=0;

    /**
     * @return An instance of std::string of the backend type. If the implementation
     * does not use a specific backend returns bitrl::consts::INVALID_STR
     */
    virtual std::string backend_type_str()const=0;

    /**
     * @return An instance of std::string indicating the units the sensor is using.
     * If units have not been established this returns bitrl::consts::INVALID_STR
     */
    virtual std::string sensor_units()const=0;

    /**
     * @brief Returns true if the sensor is enabled
     * @return
     */
    bool is_enabled()const noexcept{return is_enabled_;}

    /**
     * @brief Set the is_enabled_ flag to true
     */
    void enable()noexcept{is_enabled_=true;}

    /**
     * @brief Disable the sensor
     */
    void disable()noexcept{is_enabled_=false;}

    /**
     * @return Read reference to the last values read by the sensor
     */
    const std::vector<real_t>& last_read_values()const noexcept{return values_;}

    /**
     * @return An instance of std::string with the name of the sensor
     */
    std::string sensor_name()const noexcept{return sensor_name_;}

    /**
     * @brief Set the sensor name
     * @param sensor_name The sensor name to set
     */
    void set_sensor_name(const std::string& sensor_name)noexcept{sensor_name_=sensor_name;}

    /**
     * @return An instance of std::string with the type of the sensor
     */
    std::string sensor_type()const noexcept{return sensor_type_;}

protected:
    /**
     * @param sensor_type The type of the sensor
     */
    explicit SensorBase(const std::string& sensor_type,
                        const std::string& sensor_name=bitrl::consts::INVALID_STR);

    /**
     * @brief The values last read by the sensor
     */
    std::vector<real_t> values_;

private:

    /**
     * @brief The type of the sensor
     */
    const std::string sensor_type_;

    /**
     * @brief The name of the sensor
     */
    std::string sensor_name_{bitrl::consts::INVALID_STR};

    /**
     * @brief Flag indicating if the sensor is enabled
     */
    bool is_enabled_{false};
};

inline SensorBase::SensorBase(const std::string &sensor_type, const std::string& sensor_name)
    :
values_(),
sensor_type_(sensor_type),
sensor_name_(sensor_name)
{}


}
}

#endif //SENSOR_BASE_H
