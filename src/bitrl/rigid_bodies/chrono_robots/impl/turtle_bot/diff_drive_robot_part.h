
#ifndef DIFF_DRIVE_ROBOT_PART_H
#define DIFF_DRIVE_ROBOT_PART_H

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include <string>
#include <memory>

#include "bitrl/bitrl_types.h"
#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

namespace bitrl{
namespace rb::bitrl_chrono{

enum class CollisionFamily : int_t
{
    CHASSIS = 1,        ///< chassis
    ACTIVE_WHEEL = 2,   ///< active cylinderical drive wheel
    PASSIVE_WHEEL = 3,  ///< passive cylinderical wheel
    ROD = 4,            ///< short and long supporting rods
    BOTTOM_PLATE = 5,   ///< bottom plate
    MIDDLE_PLATE = 6,   ///< middle plate
    TOP_PLATE = 7       ///< top plate
};

class CHRONO_DiffDriveRobot_Part {
public:
    CHRONO_DiffDriveRobot_Part(const std::string& name,
                   bool fixed,
                   std::shared_ptr<chrono::ChContactMaterial> mat,
                   chrono::ChSystem* system,
                   const chrono::ChVector3d& body_pos,
                   const chrono::ChQuaternion<>& body_rot,
                   std::shared_ptr<chrono::ChBodyAuxRef> chassis_body,
                   bool collide);

    /// destructor
    virtual ~CHRONO_DiffDriveRobot_Part()=default;

    /// Return the name of the part.
    const std::string& get_name() const { return m_name; }

    /// Set the name of the part.
    void set_name(const std::string& name) { m_name = name; }

    /// Return the ChBody of the corresponding Turtlebot part.
    std::shared_ptr<chrono::ChBodyAuxRef> get_body_ptr() const { return m_body; }

    /// Return the ChBody of the chassis wrt the Turtlebot part.
    std::shared_ptr<chrono::ChBodyAuxRef> get_chassis_ptr() const { return m_chassis; }

    /// Return the Position of the Turtlebot part.
    const chrono::ChVector3d& get_pos() const { return m_body->GetFrameRefToAbs().GetPos(); }

    /// Return the Rotation of the Turtlebot part.
    const chrono::ChQuaternion<>& get_rotation() const { return m_body->GetFrameRefToAbs().GetRot(); }

protected:
    /// Initialize the visulization mesh of the Turtlebot part.
    void add_visualization_assets();

    /// Initialize the collision mesh of the Turtlebot part.
    void add_collision_shapes();

    /// Enable/disable collision.
    void enable_collision(bool state);

    std::shared_ptr<chrono::ChBodyAuxRef> m_body;      ///< rigid body
    std::shared_ptr<chrono::ChContactMaterial> m_mat;  ///< contact material (shared among all shapes)
    std::shared_ptr<chrono::ChBodyAuxRef> m_chassis;  ///< the chassis body for the robot

    chrono::ChVector3d m_pos;      ///< Turtlebot part's relative position wrt the chassis
    chrono::ChVector3d m_offset;                      ///< offset for visualization mesh
    chrono::ChColor m_color;                          ///< visualization asset color
    chrono::ChSystem* m_system;                       ///< system which Turtlebot Part belongs to

    chrono::ChQuaternion<> m_rot;  ///< Turtlebot part's relative rotation wrt the chassis

    std::string m_name;                        ///< subsystem name
    std::string m_mesh_name;                  ///< visualization mesh name
    real_t m_density;      ///< Turtlebot part's density

    bool m_collide;  ///< Turtlebot part's collision indicator
    bool m_fixed;    ///< Turtlebot part's fixed indication
};

}
}



#endif
#endif //DIFF_DRIVE_ROBOT_PART_H
