#ifndef TURTLE_ROBOT_RODS_H
#define TURTLE_ROBOT_RODS_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include <string>
#include <memory>

#include "chrono/physics/ChSystem.h"

#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/turtle_robot_part.h"

namespace bitrl
{
namespace rb::bitrl_chrono
{

/**
 * @class CHRONO_DiffDriveRobot_Rod_Short
 * @ingroup rb_chrono
 * @brief Class for representing the short rods of the DiffDriveRobot.
 * This class is a copy of the implementation provided by Chrono with
 * a few additions.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */
class CHRONO_DiffDriveRobot_Rod_Short : public CHRONO_DiffDriveRobot_Part {
public:
    CHRONO_DiffDriveRobot_Rod_Short(const std::string& name,
                        std::shared_ptr<chrono::ChContactMaterial> mat,
                        chrono::ChSystem* system,
                        const chrono::ChVector3d& body_pos,
                        const chrono::ChQuaternion<>& body_rot,
                        std::shared_ptr<chrono::ChBodyAuxRef> chassis);

    ~CHRONO_DiffDriveRobot_Rod_Short()=default;

    /// Initialize the wheel at the specified (absolute) position.
    virtual void init()override;

private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);

};

/**
 * @class CHRONO_DiffDriveRobot_Rod_Long
 * @ingroup rb_chrono
 * @brief Class for representing the long rods of the DiffDriveRobot.
 * This class is a copy of the implementation provided by Chrono with
 * a few additions.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */
class CHRONO_DiffDriveRobot_Rod_Long : public CHRONO_DiffDriveRobot_Part {
public:
    CHRONO_DiffDriveRobot_Rod_Long(const std::string& name,
                       std::shared_ptr<chrono::ChContactMaterial> mat,
                       chrono::ChSystem* system,
                       const chrono::ChVector3d& body_pos,
                       const chrono::ChQuaternion<>& body_rot,
                       std::shared_ptr<chrono::ChBodyAuxRef> chassis);

    virtual ~CHRONO_DiffDriveRobot_Rod_Long()=default;

    /// Initialize the wheel at the specified (absolute) position.
    virtual void init() override;



private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);
};
}
}
#endif
#endif //DIFF_DRIVE_ROBOT_ROD_SHORT_H
