#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "bitrl/bitrl_types.h"
#include "bitrl/sensors/sensor_base.h"
#include <memory>
#include <vector>
#include <boost/noncopyable.hpp>

namespace bitrl
{
namespace sensors
{

/**
 * @class SensorManager
 * @ingroup bitrl_sensors
 * @brief Class for managing sensor updates
 *
 */
class SensorManager: private boost::noncopyable
{
public:

    explicit SensorManager(uint_t n_sensors);

    /**
     * @brief Add a new sensor to manage
     * @param sensor
     */
    void add(std::shared_ptr<SensorBase> sensor);

    /**
     * @brief Update all the enabled sensors
     */
    void update();

    /**
     * @return The number of sensors the manager handles
     */
    uint_t n_sensors() const { return sensors_.size(); }

private:

    uint_t n_sensors_;

    /**
     * @brief The vailable sensors this manager has
     */
    std::vector< std::shared_ptr<SensorBase>> sensors_;

};
}
}


#endif //SENSOR_MANAGER_H
