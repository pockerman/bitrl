#ifndef DIFF_DRIVE_ROBOT_RODS_H
#define DIFF_DRIVE_ROBOT_RODS_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include <string>
#include <memory>

#include "chrono/physics/ChSystem.h"

#include "diff_drive_robot_part.h"

namespace bitrl
{
namespace rb::bitrl_chrono
{

/// Short Supporting Rod class definition
class CHRONO_DiffDriveRobot_Rod_Short : public CHRONO_DiffDriveRobot_Part {
public:
    CHRONO_DiffDriveRobot_Rod_Short(const std::string& name,
                        bool fixed,
                        std::shared_ptr<chrono::ChContactMaterial> mat,
                        chrono::ChSystem* system,
                        const chrono::ChVector3d& body_pos,
                        const chrono::ChQuaternion<>& body_rot,
                        std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                        bool collide);
    ~CHRONO_DiffDriveRobot_Rod_Short()=default;

    /// Initialize the wheel at the specified (absolute) position.
    void init();

    /// Enable/disable collision for the wheel.
    void enable_collision(bool state);

private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);

};


/// Long Supporting Rod class definition
class CHRONO_DiffDriveRobot_Rod_Long : public CHRONO_DiffDriveRobot_Part {
public:
    CHRONO_DiffDriveRobot_Rod_Long(const std::string& name,
                       bool fixed,
                       std::shared_ptr<chrono::ChContactMaterial> mat,
                       chrono::ChSystem* system,
                       const chrono::ChVector3d& body_pos,
                       const chrono::ChQuaternion<>& body_rot,
                       std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                       bool collide);
    virtual ~CHRONO_DiffDriveRobot_Rod_Long()=default;

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
#endif
#endif //DIFF_DRIVE_ROBOT_ROD_SHORT_H
