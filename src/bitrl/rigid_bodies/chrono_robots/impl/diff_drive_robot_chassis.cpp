#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "bitrl/rigid_bodies/chrono_robots/impl/diff_drive_robot_chassis.h"

namespace bitrl{
namespace rb::bitrl_chrono
{

CHRONO_DiffDriveRobot_Chassis::CHRONO_DiffDriveRobot_Chassis(const std::string& name,
                      bool fixed,
                      std::shared_ptr<chrono::ChContactMaterial> mat,
                      chrono::ChSystem* system,
                      const chrono::ChVector3d& body_pos,
                      const chrono::ChQuaternion<>& body_rot,
                      bool collide)
    :
    CHRONO_DiffDriveRobot_Part(name, fixed, mat, system, body_pos, body_rot, NULL, collide)
{
    this -> m_mesh_name = "chassis";
    this -> m_offset = chrono::ChVector3d(0, 0, 0);
    this -> m_color = chrono::ChColor(0.4f, 0.4f, 0.7f);
    this -> m_density = 100;
}

void CHRONO_DiffDriveRobot_Chassis::init() {
    auto vis_mesh_file = chrono::GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(chrono::ChVector3d(0, 0, 0), chrono::ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    chrono::ChVector3d mcog;
    chrono::ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    chrono::ChMatrix33<> principal_inertia_rot;
    chrono::ChVector3d principal_I;
    chrono::ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrameCOMToRef(chrono::ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);
    m_body->SetFrameRefToAbs(chrono::ChFrame<>(m_pos, m_rot));
    m_body->SetFixed(m_fixed);

    this -> add_collision_shapes();

    m_body->GetCollisionModel()->SetFamily(static_cast<int_t>(CollisionFamily::CHASSIS));
    m_body->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::ACTIVE_WHEEL));

    this -> add_visualization_assets();
    m_system->Add(m_body);
}

void CHRONO_DiffDriveRobot_Chassis::enable_collision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void CHRONO_DiffDriveRobot_Chassis::translate(const chrono::ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}


}
}

#endif
