
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
// Turtlebot Robot Class
// This is a modified version of the famous turtlebot 2e
// The geometries use the following resources as references:
// https://groups.google.com/g/sydney_ros/c/z05uQTCuDTQ
// https://grabcad.com/library/interbotix-turtlebot-2i-1
// https://www.turtlebot.com/turtlebot2/
//
// =============================================================================

#ifndef DIFF_DRIVE_ROBOT_H
#define DIFF_DRIVE_ROBOT_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO


#include "bitrl/bitrl_types.h"
#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/diff_drive_robot_chassis.h"
#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/diff_drive_robot_wheels.h"
#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/diff_drive_robot_rods.h"
#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/diff_drive_robot_plates.h"

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include <array>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>


namespace bitrl
{
namespace rb::bitrl_chrono{



/// Turtlebot Robot class
/// This class assemble and initialize a complete turtlebot robot
/// This class also handles general control commands of the robot
/**
 * @class CHRONO_DiffDriveRobotBase
 */

/**
 * @class CHRONO_DiffDriveRobotBase
 * @ingroup rb_chrono
 * @brief Class for modelling a differential-drive robot using Chrono.
 * in fact the implementation of this class is taken from the
 * Chrono::TurtleBot.
 *
 * for more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 *
 */
class CHRONO_DiffDriveRobot {
public:
    CHRONO_DiffDriveRobot(chrono::ChSystem& system,
              const chrono::ChVector3d& robot_pos,
              const chrono::ChQuaternion<>& robot_rot,
              std::shared_ptr<chrono::ChContactMaterial> wheel_mat = {});

    virtual ~CHRONO_DiffDriveRobot()=default;

    /// Initialize the turtlebot robot using current parameters.
    void init();

    /// Set active drive wheel speed
    void set_motor_speed(real_t rad_speed, uint_t id);

    /// Get active drive wheel speed
    chrono::ChVector3d get_active_wheel_speed(uint_t id) const;

    /// Get active driver wheel angular velocity
    chrono::ChVector3d get_active_wheel_angular_velocity(uint_t id)const;

private:
    /// This function initializes all parameters for the robot.
    /// Note: The robot will not be constructed in the ChSystem until Initialize() is called.
    void create();

    chrono::ChSystem* m_system;  ///< pointer to the Chrono system

    bool m_dc_motor_control = false;

    std::shared_ptr<CHRONO_DiffDriveRobot_Chassis> m_chassis;                           ///< robot chassis
    std::vector<std::shared_ptr<CHRONO_DiffDriveRobot_ActiveWheel>> m_drive_wheels;     ///< 2 active robot drive wheels
    std::vector<std::shared_ptr<CHRONO_DiffDriveRobot_PassiveWheel>> m_passive_wheels;  ///< 2 passive robot driven wheels

    std::vector<std::shared_ptr<CHRONO_DiffDriveRobot_Rod_Short>> m_1st_level_rods;  ///< six first level supporting short rods
    std::vector<std::shared_ptr<CHRONO_DiffDriveRobot_Rod_Short>> m_2nd_level_rods;  ///< six second level supporting short rods
    std::vector<std::shared_ptr<CHRONO_DiffDriveRobot_Rod_Long>> m_3rd_level_rods;   ///< six third level support long rods
    std::shared_ptr<CHRONO_DiffDriveRobot_BottomPlate> m_bottom_plate;               ///< bottom plate of the turtlebot robot
    std::shared_ptr<CHRONO_DiffDriveRobot_MiddlePlate> m_middle_plate;               ///< middle plate of the turtlebot robot
    std::shared_ptr<CHRONO_DiffDriveRobot_TopPlate> m_top_plate;                     ///< top plate of the turtlebot robot

    chrono::ChQuaternion<> m_robot_rot;  ///< robot rotation
    chrono::ChVector3d m_robot_pos;      ///< robot translation position

    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationSpeed>> m_motors;  ///< vector to store motors

    std::vector<std::shared_ptr<chrono::ChFunctionConst>> m_motors_func;  ///< constant motor angular speed func

    // model parts material
    std::shared_ptr<chrono::ChContactMaterial> m_chassis_material;  ///< chassis contact material
    std::shared_ptr<chrono::ChContactMaterial> m_wheel_material;    ///< wheel contact material (shared across limbs)
};


}  // namespace chrono
}
#endif
#endif

