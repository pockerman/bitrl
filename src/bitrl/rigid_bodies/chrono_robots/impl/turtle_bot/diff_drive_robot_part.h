
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

/**
 * @class CHRONO_DiffDriveRobot_Part
 * @ingroup rb_chrono
 * @brief Base class for modelling the various parts of the DiffDriveRobot.
 * This class is a copy of the implementation provided by Chrono with
 * a few additions.
 *
 * For more details see here: https://api.projectchrono.org/group__robot__models__turtlebot.html
 */

class CHRONO_DiffDriveRobot_Part {
public:


    /// destructor
    virtual ~CHRONO_DiffDriveRobot_Part()=default;

    /// Initialize the chassis at the specified (absolute) position.
    virtual void init()=0;

    /// Return the name of the part.
    const std::string& get_name() const { return name_; }

    /// Set the name of the part.
    void set_name(const std::string& name) { name_ = name; }

    /// Set the mesh file name
    void set_mesh_name(const std::string& name) { mesh_name_ = name; }

    /// Get the mesh file name
    std::string get_mesh_name() const { return mesh_name_; }

    /// Returns the full path for the mesh visualization.
    /// File root is consts::ROBOTS_DIR
    std::string get_vis_mesh_file()const;

    /// Return the density of this part
    real_t get_density()const{return density_; }

    /// Set the density
    void set_density(real_t density) { density_ = density; }

    /// Return the ChBody of the corresponding Turtlebot part.
    std::shared_ptr<chrono::ChBodyAuxRef> get_body_ptr() const { return body_; }

    /// Return the ChBody of the chassis wrt the Turtlebot part.
    std::shared_ptr<chrono::ChBodyAuxRef> get_chassis_ptr() const { return chassis_; }

    /// Return the Position of the Turtlebot part.
    const chrono::ChVector3d& get_pos() const { return body_->GetFrameRefToAbs().GetPos(); }

    /// Return the Rotation of the Turtlebot part.
    const chrono::ChQuaternion<>& get_rotation() const { return body_->GetFrameRefToAbs().GetRot(); }

    /// Enable/disable collision.
    void enable_collision(bool state);

protected:

    CHRONO_DiffDriveRobot_Part(const std::string& name,
                  bool fixed,
                  std::shared_ptr<chrono::ChContactMaterial> mat,
                  chrono::ChSystem* system,
                  const chrono::ChVector3d& body_pos,
                  const chrono::ChQuaternion<>& body_rot,
                  std::shared_ptr<chrono::ChBodyAuxRef> chassis_body,
                  bool collide);

    // Initialize the body_ Called upon constructing
    // an instance of this class
    void do_init_(const std::string& mesh_name, const chrono::ChVector3d& offset,
        const chrono::ChColor& color, real_t density);

    /// Initialize the visulization mesh of the Turtlebot part.
    void add_visualization_assets();

    /// Initialize the collision mesh of the Turtlebot part.
    void add_collision_shapes();

    std::shared_ptr<chrono::ChBodyAuxRef> body_;      ///< rigid body
    std::shared_ptr<chrono::ChContactMaterial> contact_material_;  ///< contact material (shared among all shapes)
    std::shared_ptr<chrono::ChBodyAuxRef> chassis_;  ///< the chassis body for the robot

    chrono::ChVector3d pos_;      ///< Turtlebot part's relative position wrt the chassis
    chrono::ChVector3d offset_;                      ///< offset for visualization mesh
    chrono::ChColor color_;                          ///< visualization asset color
    chrono::ChSystem* system_;                       ///< system which Turtlebot Part belongs to

    chrono::ChQuaternion<> rot_;  ///< Turtlebot part's relative rotation wrt the chassis

    std::string name_;                        ///< subsystem name
    std::string mesh_name_;                  ///< visualization mesh name
    real_t density_;      ///< Turtlebot part's density

    bool collide_;  ///< Turtlebot part's collision indicator
    bool fixed_;    ///< Turtlebot part's fixed indication
    bool is_initialized_{false};
};

}
}



#endif
#endif //DIFF_DRIVE_ROBOT_PART_H
