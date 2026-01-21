#ifndef CHRONO_ULTRASOUND_BACKEND_H
#define CHRONO_ULTRASOUND_BACKEND_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/sensors/backends/range_sensor_backend_base.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"

#include <memory>
#include <string>

namespace bitrl
{
namespace sensors::backends
{

/**
 * @class CHRONO_UltrasonicBackend
 * @ingroup bitrl_sensors_backends
 * @brief Class for modelling ultrasonic sensors
 */
class CHRONO_UltrasonicBackend final: public RangeSensorBackendBase
{
public:

    /**
     * @brief The type of the sensor
     */
    static const  std::string BACKEND_TYPE;

    /**
     * @brief Constructor
     */
    CHRONO_UltrasonicBackend(chrono::ChSystem& sys_ptr,
                            std::shared_ptr<chrono::ChBody> body);

    /**
     * @brief Read the sensor value
     * @return
     */
    std::vector<real_t> read_values();

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
    void load_from_json(const std::string& filename);

    /**
     * @brief Set the position of the sensor
     * @param pos
     */
    void set_position(const chrono::ChVector3d& pos){position_ = pos;}

    /**
     * @return A read reference of the position of the sensor
     */
    const chrono::ChVector3d& position() const{return position_;}


private:

    /**
     * @brief Pointer to the system this sensor is attached ot
     */
    chrono::ChSystem* sys_ptr_;

    /**
     * @brief The body on which the sensor sits
     */
    std::shared_ptr<chrono::ChBody> body_;

    /**
     * @brief The position of the sensor
     */
    chrono::ChVector3d position_;

};
}
}

#endif
#endif //CHRONO_ULTRASOUND_BACKEND_H
