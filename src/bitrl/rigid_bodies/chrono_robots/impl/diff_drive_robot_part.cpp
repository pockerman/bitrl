#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/rigid_bodies/chrono_robots/impl/diff_drive_robot_part.h"

namespace bitrl{
namespace rb::bitrl_chrono{

CHRONO_DiffDriveRobot_Part::CHRONO_DiffDriveRobot_Part(const std::string& name,
                   bool fixed,
                   std::shared_ptr<chrono::ChContactMaterial> mat,
                   chrono::ChSystem* system,
                   const chrono::ChVector3d& body_pos,
                   const chrono::ChQuaternion<>& body_rot,
                   std::shared_ptr<chrono::ChBodyAuxRef> chassis_body,
                   bool collide){
    m_body = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    m_body->SetName(name + "_body");
    m_chassis = chassis_body;
    m_mat = mat;
    m_pos = body_pos;
    m_rot = body_rot;
    m_system = system;
    m_collide = collide;
    m_fixed = fixed;
}

// Create Visulization assets
void CHRONO_DiffDriveRobot_Part::add_visualization_assets() {
    auto vis_mesh_file = chrono::GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
    trimesh->Transform(m_offset, chrono::ChMatrix33<>(1));
    auto trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(m_mesh_name);
    m_body->AddVisualShape(trimesh_shape);
    return;
}

void CHRONO_DiffDriveRobot_Part::enable_collision(bool state) {
    m_collide = state;
}

// Add collision assets
void CHRONO_DiffDriveRobot_Part::add_collision_shapes() {
    auto vis_mesh_file = chrono::GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(m_offset, chrono::ChMatrix33<>(1));

    auto shape = chrono_types::make_shared<chrono::ChCollisionShapeTriangleMesh>(m_mat, trimesh, false, false, 0.005);
    m_body->AddCollisionShape(shape);
    m_body->EnableCollision(m_collide);
}

}
}
#endif