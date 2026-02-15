#ifndef DIFF_DRIVE_ROBOT_PLATES_H
#define DIFF_DRIVE_ROBOT_PLATES_H

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
/// Turtlebot Bottom Plate class definition
class CHRONO_DiffDriveRobot_BottomPlate : public CHRONO_DiffDriveRobot_Part {
  public:
    CHRONO_DiffDriveRobot_BottomPlate(const std::string& name,
                          bool fixed,
                          std::shared_ptr<chrono::ChContactMaterial> mat,
                          chrono::ChSystem* system,
                          const chrono::ChVector3d& body_pos,
                          const chrono::ChQuaternion<>& body_rot,
                          std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                          bool collide);
    virtual ~CHRONO_DiffDriveRobot_BottomPlate()=default;

    /// Initialize the wheel at the specified (absolute) position.
    void init();

    /// Enable/disable collision for the wheel.
    void enable_collision(bool state);

  private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);

};

/// Turtlebot Middle Plate class definition
class CHRONO_DiffDriveRobot_MiddlePlate : public CHRONO_DiffDriveRobot_Part {
  public:
    CHRONO_DiffDriveRobot_MiddlePlate(const std::string& name,
                          bool fixed,
                          std::shared_ptr<chrono::ChContactMaterial> mat,
                          chrono::ChSystem* system,
                          const chrono::ChVector3d& body_pos,
                          const chrono::ChQuaternion<>& body_rot,
                          std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                          bool collide);
    virtual ~CHRONO_DiffDriveRobot_MiddlePlate()=default;

    /// Initialize the wheel at the specified (absolute) position.
    void init();

    /// Enable/disable collision for the wheel.
    void enable_collision(bool state);

  private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);

};

/// Turtlebot Top Plate class definition
class CHRONO_DiffDriveRobot_TopPlate : public CHRONO_DiffDriveRobot_Part {
  public:
    CHRONO_DiffDriveRobot_TopPlate(const std::string& name,
                       bool fixed,
                       std::shared_ptr<chrono::ChContactMaterial> mat,
                       chrono::ChSystem* system,
                       const chrono::ChVector3d& body_pos,
                       const chrono::ChQuaternion<>& body_rot,
                       std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                       bool collide);
    virtual ~CHRONO_DiffDriveRobot_TopPlate() {}

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

#endif //DIFF_DRIVE_ROBOT_PLATES_H
