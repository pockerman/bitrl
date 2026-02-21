#ifndef CHRONO_DIFF_DRIVE_ROBOT_H
#define CHRONO_DIFF_DRIVE_ROBOT_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"
#include "bitrl/sensors/sensor_manager.h"
#include "bitrl/rigid_bodies/chrono_robots/chrono_robot_pose.h"


#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChLinkMotorRotationSpeed.h>


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

    typedef CHRONO_RobotPose pose_type;

    /**
     * Handle the motors
     */
    struct MotorHandle {
        std::shared_ptr<chrono::ChLinkMotorRotationSpeed> motor;
        std::shared_ptr<chrono::ChFunctionConst> speed;
    };


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
     * @brief Step the underlying chrono::ChSystemSMC one time step
     * @param time_step
     */
    void step(real_t time_step);

    /**
     * @brief Set the motor speed
     * @param motor_name The name of the motor
     * @param speed The speed
     */
    void set_motor_speed(const std::string& motor_name, real_t speed);

    /**
     * @brief Set the speed for both motors
     * @param speed
     */
    void set_motor_speed(real_t speed);

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

    /**
     * Set the pointer to the sensor manager
     * @param sensors_manager
     */
    void set_sensors(sensors::SensorManager& sensor_manager){sensor_manager_ptr_ = &sensor_manager;}

    /**
     * @return
     */
    chrono::ChSystemSMC& CHRONO_sys() noexcept{return sys_;}


    std::shared_ptr<chrono::ChBody> CHRONO_chassis()noexcept{return chassis_;}

    /**
     * @return Pointer to the state of the robot
     */
    std::shared_ptr<pose_type> pose()noexcept{return pose_;}

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
     * The chassis of the robot
     */
    std::shared_ptr<chrono::ChBody> chassis_;

    /**
     * The state of the robot
     */
    std::shared_ptr<pose_type> pose_;

    /**
     * @brief Manages the sensors on the robot
     */
    sensors::SensorManager* sensor_manager_ptr_;

    /**
     * @brief Motors for the robot
     * motors_.first left motor
     * motors.second right motor
     */
    std::pair<MotorHandle, MotorHandle> motors_;

    /**
     * @brief The name of the robot
     */
    std::string name_;

};

inline void CHRONO_DiffDriveRobotBase::set_motor_speed(real_t speed)
{
    set_motor_speed("left_motor", speed);
    set_motor_speed("right_motor", speed);
}
}
}
#endif
#endif //DIFF_DRIVE_ROBOT_H
