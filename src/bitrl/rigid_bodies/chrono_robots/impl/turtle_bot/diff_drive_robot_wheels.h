#ifndef DIFF_DRIVE_ROBOT_WHEELS_H
#define DIFF_DRIVE_ROBOT_WHEELS_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include <string>
#include <memory>

#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/diff_drive_robot_part.h"
namespace bitrl{
namespace rb::bitrl_chrono
{

/**
 * @class CHRONO_DiffDriveRobot_ActiveWheel
 * @ingroup rb_chrono
 * @brief Class for representing motorised wheel of the DiffDriveRobot.
 * This class is a copy of the implementation provided by Chrono with
 * a few additions.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */
class CHRONO_DiffDriveRobot_ActiveWheel : public CHRONO_DiffDriveRobot_Part {
public:
    CHRONO_DiffDriveRobot_ActiveWheel(const std::string& name,
                          std::shared_ptr<chrono::ChContactMaterial> mat,
                          chrono::ChSystem* system,
                          const chrono::ChVector3d& body_pos,
                          const chrono::ChQuaternion<>& body_rot,
                          std::shared_ptr<chrono::ChBodyAuxRef> chassis
                         );

    // destructor
    virtual ~CHRONO_DiffDriveRobot_ActiveWheel()=default;

    /// Initialize the wheel at the specified (absolute) position.
    virtual void init() override;

private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);
};

/**
 * @class CHRONO_DiffDriveRobot_PassiveWheel
 * @ingroup rb_chrono
 * @brief Class for representing a passive wheel of the DiffDriveRobot.
 * This class is a copy of the implementation provided by Chrono with
 * a few additions.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */
class CHRONO_DiffDriveRobot_PassiveWheel : public CHRONO_DiffDriveRobot_Part {
public:
    CHRONO_DiffDriveRobot_PassiveWheel(const std::string& name,

                           std::shared_ptr<chrono::ChContactMaterial> mat,
                           chrono::ChSystem* system,
                           const chrono::ChVector3d& body_pos,
                           const chrono::ChQuaternion<>& body_rot,
                           std::shared_ptr<chrono::ChBodyAuxRef> chassis
                           );
    virtual ~CHRONO_DiffDriveRobot_PassiveWheel()=default;

    /// Initialize the wheel at the specified (absolute) position.
    virtual void init() override;

private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);

};

}
}

#endif //DIFF_DRIVE_ROBOT_ACTIVE_WHEEL_H
#endif
