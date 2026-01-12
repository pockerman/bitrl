#ifndef CHRONO_DIFF_DRIVE_ROBOT_H
#define CHRONO_DIFF_DRIVE_ROBOT_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"
#include "chrono/physics/ChSystemSMC.h"

#include <string>
#include <memory>
#include <utility>

namespace bitrl
{
namespace rb::bitrl_chrono
{
/**
 * @class CHRONO_DiffDriveRobotBase
 * @ingroup rb_chrono
 * @brief Base class for a differential-drive robot using Project Chrono.
 *
 * This class provides a common foundation for modeling and simulating
 * differential-drive robots using Chrono physics objects. It encapsulates
 * a Chrono simulation system and supports initialization from an external
 * JSON configuration file.
 *
 * The class can be instantiated directly for simple simulations or
 * extended to implement more specialized robot behaviors.
 *
 * @note This class assumes the use of Chrono's SMC contact model.
 */
class CHRONO_DiffDriveRobotBase
{
public:

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
     * @brief The name of the robot
     * @return
     */
    const std::string& get_name() const noexcept{return name_;}

    /**
     * @brief Retruns the number of wheels this robot has
     * @return
     */
    uint_t n_wheels()const noexcept{return 3;}

    /**
     * @brief Returns the number of motors this robot has
     * @return
     */
    uint_t n_motors()const noexcept{return 2;}

protected:

    /**
     * @brief Chrono physics system used for simulation.
     *
     * This system owns and manages all physical bodies, constraints,
     * and contact interactions associated with the robot and the
     * environment.
     */
    chrono::ChSystemSMC sys_;

    /**
     * @brief Motors for the robot
     * motors_.first left motor
     * motors.second right motor
     */
    std::pair<std::shared_ptr<chrono::ChBody>, std::shared_ptr<chrono::ChBody>> motors_;

    /**
     * @brief The name of the robot
     */
    std::string name_;


};
}
}
#endif
#endif //DIFF_DRIVE_ROBOT_H
