#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "chrono/assets/ChColor.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/rigid_bodies/chrono_robots/impl/diff_drive_robot_active_wheel.h"


namespace bitrl
{
namespace rb::bitrl_chrono
{

CHRONO_DiffDriveRobot_ActiveWheel::CHRONO_DiffDriveRobot_ActiveWheel(const std::string& name,
                                             bool fixed,
                                             std::shared_ptr<chrono::ChContactMaterial> mat,
                                             chrono::ChSystem* system,
                                             const chrono::ChVector3d& body_pos,
                                             const chrono::ChQuaternion<>& body_rot,
                                             std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                                             bool collide)
    :
    CHRONO_DiffDriveRobot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide)
{
    m_mesh_name = "active_wheel";
    m_offset = chrono::ChVector3d(0, 0, 0);
    m_color = chrono::ChColor(0.4f, 0.4f, 0.7f);
    m_density = 200;
}

void CHRONO_DiffDriveRobot_ActiveWheel::init() {
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

    // set relative position to chassis
    const chrono::ChFrame<>& X_GP = m_chassis->GetFrameRefToAbs();  // global -> parent
    chrono::ChFrame<> X_PC(m_pos, m_rot);                           // parent -> child
    chrono::ChFrame<> X_GC = X_GP * X_PC;                           // global -> child
    m_body->SetFrameRefToAbs(X_GC);
    m_body->SetFixed(m_fixed);

    this -> add_collision_shapes();

    m_body->GetCollisionModel()->SetFamily(static_cast<int_t>(CollisionFamily::ACTIVE_WHEEL));
    m_body->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::CHASSIS));

    this -> add_visualization_assets();

    m_system->Add(m_body);
}

void CHRONO_DiffDriveRobot_ActiveWheel::enable_collision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void CHRONO_DiffDriveRobot_ActiveWheel::translate(const chrono::ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}


}
}
#endif