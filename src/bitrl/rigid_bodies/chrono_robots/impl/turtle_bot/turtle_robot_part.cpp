#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/turtle_robot_part.h"

namespace bitrl{
namespace rb::bitrl_chrono{

CHRONO_DiffDriveRobot_Part::CHRONO_DiffDriveRobot_Part(const std::string& name,
                   bool fixed,
                   std::shared_ptr<chrono::ChContactMaterial> mat,
                   chrono::ChSystem* system,
                   const chrono::ChVector3d& body_pos,
                   const chrono::ChQuaternion<>& body_rot,
                   std::shared_ptr<chrono::ChBodyAuxRef> chassis_body,
                   bool collide)
                       :
body_(nullptr),
contact_material_(mat),
chassis_(chassis_body),
pos_(body_pos),
rot_(body_rot),
system_(system),
name_(name),
mesh_name_(bitrl::consts::INVALID_STR),
density_(1.0),
collide_(collide),
fixed_(fixed),
is_initialized_(false)
{
    //init_body_(name);
    // body_ = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    // body_->SetName(name + "_body");
    // chassis_ = chassis_body;
    // contact_material_ = mat;
    // pos_ = body_pos;
    // rot_ = body_rot;
    // system_ = system;
    // collide_ = collide;
    // fixed_ = fixed;
}


void CHRONO_DiffDriveRobot_Part::do_init_(const std::string& mesh_name, const chrono::ChVector3d& offset,
                                          const chrono::ChColor& color, real_t density)
{
    body_ = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    body_->SetName(name_ + "_body");
    set_density(density);
    set_mesh_name(mesh_name);
    color_ = color;
    offset_ = offset;
}

std::string CHRONO_DiffDriveRobot_Part::get_vis_mesh_file()const
{
    if (mesh_name_ == bitrl::consts::INVALID_STR)
    {
        throw std::runtime_error("The mesh name file is not specified");
    }

    return std::string(consts::ROBOTS_DIR + "/diff_drive_robot/" + mesh_name_ + ".obj");
}

// Create Visulization assets
void CHRONO_DiffDriveRobot_Part::add_visualization_assets() {
    const std::string vis_mesh_file = get_vis_mesh_file();

    auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
    trimesh->Transform(offset_, chrono::ChMatrix33<>(1));
    auto trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(mesh_name_);
    body_->AddVisualShape(trimesh_shape);
    return;
}

void CHRONO_DiffDriveRobot_Part::enable_collision(bool state) {
    collide_ = state;
    body_ -> EnableCollision(state);
}

// Add collision assets
void CHRONO_DiffDriveRobot_Part::add_collision_shapes() {
    //const std::string vis_mesh_file(consts::ROBOTS_DIR + "/diff_drive_robot/" + mesh_name_ + ".obj");
    const std::string vis_mesh_file = get_vis_mesh_file();
    auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(offset_, chrono::ChMatrix33<>(1));

    auto shape = chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(contact_material_, trimesh, false, false, 0.005);
    body_->AddCollisionShape(shape);
    body_->EnableCollision(collide_);
}

}
}
#endif