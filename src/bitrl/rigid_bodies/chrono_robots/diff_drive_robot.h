
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
#include "bitrl/rigid_bodies/chrono_robots/chrono_robot_pose.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include <vector>


namespace bitrl
{
namespace rb::bitrl_chrono{


/**
 * @class CHRONO_DiffDriveRobotBase
 * @ingroup rb_chrono
 * @brief Class for modelling a differential-drive robot using Chrono.
 * in fact the implementation of this class is taken from the
 * Chrono::TurtleBot.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */
class CHRONO_DiffDriveRobot final {
public:
    CHRONO_DiffDriveRobot(chrono::ChSystem& system,
              const chrono::ChVector3d& robot_pos,
              const chrono::ChQuaternion<>& robot_rot,
              std::shared_ptr<chrono::ChContactMaterial> wheel_mat = {});


    /// destructor
    ~CHRONO_DiffDriveRobot()=default;

    /// Initialize the CHRONO_DiffDriveRobot  robot 
    /// The robot will not be constructed in the ChSystem until init() is called.
    void init();

    /// Set active drive wheel speed
    void set_motor_speed(real_t rad_speed, uint_t id);

    /// Get active drive wheel speed
    chrono::ChVector3d get_active_wheel_speed(uint_t id) const;

    /// Get active driver wheel angular velocity
    chrono::ChVector3d get_active_wheel_angular_velocity(uint_t id)const;

    /// Get the pose of the robot
    const CHRONO_RobotPose& get_pose() const{return pose_;}

private:

    /// Build motors
    void build_motors_();

    /// Pointer to the Chrono system
    chrono::ChSystem* system_;  

    /// robot chassis. We track the robot pose based on this
    std::shared_ptr<CHRONO_DiffDriveRobot_Chassis> chassis_;
    std::vector<std::shared_ptr<CHRONO_DiffDriveRobot_ActiveWheel>> drive_wheels_;     ///< 2 active robot drive wheels
    std::vector<std::shared_ptr<CHRONO_DiffDriveRobot_PassiveWheel>> passive_wheels_;  ///< 2 passive robot driven wheels

    std::vector<std::shared_ptr<CHRONO_DiffDriveRobot_Rod_Short>> m_1st_level_rods_;  ///< six first level supporting short rods
    std::vector<std::shared_ptr<CHRONO_DiffDriveRobot_Rod_Short>> m_2nd_level_rods_;  ///< six second level supporting short rods
    std::vector<std::shared_ptr<CHRONO_DiffDriveRobot_Rod_Long>> m_3rd_level_rods_;   ///< six third level support long rods
    std::shared_ptr<CHRONO_DiffDriveRobot_BottomPlate> bottom_plate_;               ///< bottom plate of the turtlebot robot
    std::shared_ptr<CHRONO_DiffDriveRobot_MiddlePlate> middle_plate_;               ///< middle plate of the turtlebot robot
    std::shared_ptr<CHRONO_DiffDriveRobot_TopPlate> top_plate_;                     ///< top plate of the turtlebot robot

    chrono::ChQuaternion<> robot_rot_;  ///< init robot rotation
    chrono::ChVector3d robot_pos_;      ///< init robot translation position

    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationSpeed>> motors_;  ///< vector to store motors
    std::vector<std::shared_ptr<chrono::ChFunctionConst>> motors_func_;  ///< constant motor angular speed func

    // model parts material
    std::shared_ptr<chrono::ChContactMaterial> chassis_material_;  ///< chassis contact material
    std::shared_ptr<chrono::ChContactMaterial> wheel_material_;

    /// The pose of the chassis
    CHRONO_RobotPose pose_;

    bool dc_motor_control_ = false;
};


}  // namespace chrono
}
#endif
#endif

