#ifndef TURTLE_ROBOT_CHASSIS_H
#define TURTLE_ROBOT_CHASSIS_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/turtle_robot_part.h"
#include <chrono/physics/ChSystem.h>


#include <memory>

namespace bitrl{
namespace rb::bitrl_chrono{


/**
 * @class CHRONO_DiffDriveRobot_Chassis
 * @ingroup rb_chrono
 * @brief Class for representing the chassi of the DiffDriveRobot.
 * This class is a copy of the implementation provided by Chrono with
 * a few additions.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */
class CHRONO_DiffDriveRobot_Chassis : public CHRONO_DiffDriveRobot_Part {
public:

    // constructor
    CHRONO_DiffDriveRobot_Chassis(
                      std::shared_ptr<chrono::ChContactMaterial> mat,
                      chrono::ChSystem* system,
                      const chrono::ChVector3d& body_pos,
                      const chrono::ChQuaternion<>& body_rot
                      );

    ~CHRONO_DiffDriveRobot_Chassis()=default;

    /// Initialize the chassis
    virtual void init() override;


private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);

};

}
}
#endif //DIFF_DRIVE_ROBOT_CHASSIS_H
#endif

