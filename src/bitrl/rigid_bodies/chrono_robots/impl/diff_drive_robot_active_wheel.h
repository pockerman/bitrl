#ifndef DIFF_DRIVE_ROBOT_ACTIVE_WHEEL_H
#define DIFF_DRIVE_ROBOT_ACTIVE_WHEEL_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include <string>
#include <memory>

#include "bitrl/rigid_bodies/chrono_robots/impl/diff_drive_robot_part.h"
namespace bitrl{
namespace rb::bitrl_chrono
{

class CHRONO_DiffDriveRobot_ActiveWheel : public CHRONO_DiffDriveRobot_Part {
public:
    CHRONO_DiffDriveRobot_ActiveWheel(const std::string& name,
                          bool fixed,
                          std::shared_ptr<chrono::ChContactMaterial> mat,
                          chrono::ChSystem* system,
                          const chrono::ChVector3d& body_pos,
                          const chrono::ChQuaternion<>& body_rot,
                          std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                          bool collide);

    // destructor
    virtual ~CHRONO_DiffDriveRobot_ActiveWheel()=default;

    /// Initialize the wheel at the specified (absolute) position.
    void init();

    /// Enable/disable collision for the wheel.
    void enable_collision(bool state);

private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);
};

}
}

#endif //DIFF_DRIVE_ROBOT_ACTIVE_WHEEL_H
#endif
