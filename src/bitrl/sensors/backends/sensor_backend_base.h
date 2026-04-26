#ifndef SENSOR_BACKEND_BASE_H
#define SENSOR_BACKEND_BASE_H

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO
#include <chrono/physics/ChBody.h>
#include <unordered_map>
#include <any>
#endif

#include <string>
#include <vector>


namespace bitrl
{
namespace sensors::backends
{

/**
 * @class SensorBackendBase
 * @ingroup bitrl_sensors_backends
 * @brief Base class for modelling sensors
 * with different implementations
 */
class SensorBackendBase
{
public:
    /**
     * @brief Destructor
     */
    virtual ~SensorBackendBase()=default;

    /**
     * @brief Returns the sampling period of the sensor
     * @return
     */
    real_t sampling_period()const noexcept {return sampling_period_;}

    /**
     * @brief Set the sampling period of the backend
     * @param period
     */
    void set_sampling_period(real_t period){sampling_period_ = period;}

    /**
     * @brief Read the sensor value
     * @return
     */
    virtual std::vector<real_t> read_values()=0;

    /**
     * @return An instance of std::string representing the units the sensor
     * readings are assumed in
     */
    virtual std::string sensor_units()const{return bitrl::consts::INVALID_STR;}

    /**
      * @brief Load robot and simulation parameters from a JSON file.
      *
      * This function initializes the robot and its associated Chrono
      * simulation objects based on the contents of the provided JSON
      * configuration file.
      *
      * Typical parameters may include:
      * - Physical dimensions
      * - Mass and inertia properties
      * - Wheel configuration
      * - Simulation settings
      *
      * @param filename Path to the JSON configuration file.
      *
      * @throws std::runtime_error If the file cannot be read or parsed.
      */
    virtual void load_from_json(const std::string& filename)=0;

    /**
     * @return The number of values the implementation returns
     */
    uint_t n_read_values()const noexcept{return n_read_values_;}

    /**
     * @bief Return a string that specifies  the type of the backend
     * @return
     */
    std::string backend_type_str()const noexcept{return backend_type_;}

#ifdef BITRL_CHRONO
    ///
    /// @brief Add a visual shape for the sensor in order to
    /// be visualized with Irrlicht. Concrete classes may choose not
    /// to implement this. In this case the sensor will not be visible
    ///
    virtual void add_visual_shape(const std::unordered_map<std::string, std::any>& shape_properties)=0;

    /// @brief Set the pointer to the body that this sensor is attached
    /// @param body
    void set_chrono_body(std::shared_ptr<chrono::ChBody> body){body_ = body;}
#endif

protected:
    /**
     * @brief Constructor
     * @param backend_type
     */
    SensorBackendBase(const std::string& backend_type);

#ifdef BITRL_CHRONO
    ///
    /// @brief The body on which the sensor sits
    ///
    std::shared_ptr<chrono::ChBody> body_{nullptr};
#endif

private:

    /**
     * @brief The backend type flag
     */
    const std::string backend_type_;


    /**
     * @brief The sampling period
     */
    real_t sampling_period_{0.0};

    /**
     * @brief How many values the backend returns
     */
    uint_t n_read_values_{1};


};

inline
SensorBackendBase::SensorBackendBase(const std::string& backend_type)
    :
backend_type_(backend_type)
{}

}
}

#endif //SENSOR_BACKEND_BASE_H
