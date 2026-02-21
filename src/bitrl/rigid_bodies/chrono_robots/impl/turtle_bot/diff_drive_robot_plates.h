#ifndef DIFF_DRIVE_ROBOT_PLATES_H
#define DIFF_DRIVE_ROBOT_PLATES_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include <string>
#include <memory>

#include "chrono/physics/ChSystem.h"

#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/diff_drive_robot_part.h"

namespace bitrl
{
namespace rb::bitrl_chrono
{

/**
 * @class CHRONO_DiffDriveRobot_BottomPlate
 * @ingroup rb_chrono
 * @brief Class for representing the bottom plate of the DiffDriveRobot.
 * This class is a copy of the implementation provided by Chrono with
 * a few additions.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */
class CHRONO_DiffDriveRobot_BottomPlate : public CHRONO_DiffDriveRobot_Part {
  public:
    CHRONO_DiffDriveRobot_BottomPlate(std::shared_ptr<chrono::ChContactMaterial> mat,
                          chrono::ChSystem* system,
                          const chrono::ChVector3d& body_pos,
                          const chrono::ChQuaternion<>& body_rot,
                          std::shared_ptr<chrono::ChBodyAuxRef> chassis);
    virtual ~CHRONO_DiffDriveRobot_BottomPlate()=default;

    /// Initialize the wheel at the specified (absolute) position.
    virtual void init() override;

  private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);

};


/**
 * @class CHRONO_DiffDriveRobot_MiddlePlate
 * @ingroup rb_chrono
 * @brief Class for representing the middle plate of the DiffDriveRobot.
 * This class is a copy of the implementation provided by Chrono with
 * a few additions.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */
class CHRONO_DiffDriveRobot_MiddlePlate : public CHRONO_DiffDriveRobot_Part {
  public:
    CHRONO_DiffDriveRobot_MiddlePlate(
                          std::shared_ptr<chrono::ChContactMaterial> mat,
                          chrono::ChSystem* system,
                          const chrono::ChVector3d& body_pos,
                          const chrono::ChQuaternion<>& body_rot,
                          std::shared_ptr<chrono::ChBodyAuxRef> chassis);

    virtual ~CHRONO_DiffDriveRobot_MiddlePlate()=default;

    /// Initialize the wheel at the specified (absolute) position.
    virtual void init()override;


  private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);

};


/**
 * @class CHRONO_DiffDriveRobot_TopPlate
 * @ingroup rb_chrono
 * @brief Class for representing the top plate of the DiffDriveRobot.
 * This class is a copy of the implementation provided by Chrono with
 * a few additions.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */
class CHRONO_DiffDriveRobot_TopPlate : public CHRONO_DiffDriveRobot_Part {
  public:

    /// Constructor
    CHRONO_DiffDriveRobot_TopPlate(std::shared_ptr<chrono::ChContactMaterial> mat,
                       chrono::ChSystem* system,
                       const chrono::ChVector3d& body_pos,
                       const chrono::ChQuaternion<>& body_rot,
                       std::shared_ptr<chrono::ChBodyAuxRef> chassis);
    virtual ~CHRONO_DiffDriveRobot_TopPlate() {}

    /// Initialize the wheel at the specified (absolute) position.
    virtual void init()override;

  private:
    /// Translate the chassis by the specified value.
    void translate(const chrono::ChVector3d& shift);

};

}
}
#endif

#endif //DIFF_DRIVE_ROBOT_PLATES_H
