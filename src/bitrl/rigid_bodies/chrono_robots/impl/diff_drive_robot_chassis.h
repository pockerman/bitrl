//
// Created by alex on 2/8/26.
//

#ifndef DIFF_DRIVE_ROBOT_CHASSIS_H
#define DIFF_DRIVE_ROBOT_CHASSIS_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include <string>
#include <memory>

#include "chrono/physics/ChSystem.h"
//#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "bitrl/rigid_bodies/chrono_robots/impl/diff_drive_robot_part.h"
#include "bitrl/bitrl_types.h"

namespace bitrl{
namespace rb::bitrl_chrono{

class CHRONO_DiffDriveRobot_Chassis : public CHRONO_DiffDriveRobot_Part {
public:
    CHRONO_DiffDriveRobot_Chassis(const std::string& name,
                      bool fixed,
                      std::shared_ptr<chrono::ChContactMaterial> mat,
                      chrono::ChSystem* system,
                      const chrono::ChVector3d& body_pos,
                      const chrono::ChQuaternion<>& body_rot,
                      bool collide);
    ~CHRONO_DiffDriveRobot_Chassis()=default;

    /// Initialize the chassis at the specified (absolute) position.
    void init();

    /// Enable/disable collision for the robot chassis.
    void enable_collision(bool state);

private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);

};

}
}
#endif //DIFF_DRIVE_ROBOT_CHASSIS_H
#endif

