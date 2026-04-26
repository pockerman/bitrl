#ifndef RANGE_SENSOR_BACKEND_BASE_H
#define RANGE_SENSOR_BACKEND_BASE_H

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/sensors/backends/sensor_backend_base.h"
#include <string>
#include <memory>

#ifdef BITRL_CHRONO
#include <unordered_map>
#include <any>
#endif

namespace bitrl
{
namespace sensors::backends
{
/**
 * @class RangeSensorBackendBase
 * @ingroup bitrl_sensors_backends
 * @brief Base class for modelling range sensors
 */
class RangeSensorBackendBase : public SensorBackendBase
{
public:


    static std::shared_ptr<RangeSensorBackendBase> create(const std::string& backend_type);

    /**
     * @return The maximum distance the range sensor can read
     */
    real_t max_distance()const noexcept {return max_distance_;}

    /**
     * @brief Set the maximum distance the range sensor can read
     * @param max_distance
     */
    void set_max_distance(real_t max_distance) noexcept {max_distance_ = max_distance;}

    /**
     * @return The maximum distance the range sensor can read
     */
    real_t min_distance()const noexcept {return min_distance_;}

    /**
     * @brief Set the maximum distance the range sensor can read
     * @param min_distance
     */
    void set_min_distance(real_t min_distance) noexcept {min_distance_ = min_distance;}

    /**
     * @return An instance of std::string representing the units the sensor
     * readings are assumed in
     */
    virtual std::string sensor_units()const{return sensor_units_;}

    /**
     * @brief Set the sensor units
     * @param units The string representing the sensor units
     */
    void set_sensor_units(const std::string& units){sensor_units_ = units;}

#ifdef BITRL_CHRONO
    ///
    /// @brief Add a visual shape for the sensor in order to
    /// be visualized with Irrlicht. Concrete classes may choose not
    /// to implement this. In this case the sensor will not be visible
    ///
    virtual void add_visual_shape(const std::unordered_map<std::string, std::any>& shape_properties)override;
#endif

protected:
    /**
     * @brief Constructor
     * @param backend_type
     */
    explicit RangeSensorBackendBase(const std::string& backend_type);

private:

    std::string sensor_units_{"METERS"};

    /**
     *@brief The maximum distance the sensor can read
     */
    real_t max_distance_{0.0};

    real_t min_distance_{0.0};

};

inline RangeSensorBackendBase::RangeSensorBackendBase(const std::string &backend_type)
    :
SensorBackendBase(backend_type)
{}

}
}


#endif //RANGE_SENSOR_BACKEND_BASE_H
