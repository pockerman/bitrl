#ifndef MQTT_ULTRASOUND_BACKEND_H
#define MQTT_ULTRASOUND_BACKEND_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_MQTT

#include "bitrl/bitrl_types.h"
#include "bitrl/network/mqtt_subscriber.h"
#include "bitrl/sensors/backends/range_sensor_backend_base.h"
#include "bitrl/sensors/messages/ultrasound.h"

namespace bitrl{
namespace sensors::backends
{

/**
 * @class ChronoUltrasonicBackend
 * @ingroup bitrl_sensors_backends
 * @brief Class for accessing Ultrasonic  sensors via MQTT
 * Note this class has no other info other than the position of
 * the sensor on the robot
 */
class MQTT_UltrasonicBackend: public RangeSensorBackendBase
{
public:

    /**
   * @brief The type of the sensor
   */
    static const  std::string BACKEND_TYPE;

    /**
     * @brief Constructor
     */
    explicit MQTT_UltrasonicBackend(network::MqttSubscriber& subscriber);

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
     * @brief Read the sensor value
     * @return
     */
    std::vector<real_t> read_values();

    /**
     * @brief Set the position of the sensor
     * @param pos
     */
    void set_position(const RealColVec3d& pos){position_ = pos;}

    /**
     * @return A read reference of the position of the sensor
     */
    const RealColVec3d& position() const{return position_;}

    /**
     * @return A read reference to the last UltrasoundMessage that was read
     */
    const UltrasoundMessage& last_value_read()const{return last_message_;}

private:

    /**
    * @brief Pointer to the instance that handles the MQTT connection
    * and message exchange
    */
    network::MqttSubscriber* subscriber_ptr_;

    /**
     * @brief The position of the sensor
     */
    RealColVec3d position_;

    /**
     * @brief The last read message
     */
    UltrasoundMessage last_message_;

    /**
     * @brief Time for polling the sensor
     */
    uint_t polling_time_milli{3000};

};

}
}
#endif
#endif //MQTT_ULTRASOUND_BACKEND_H
