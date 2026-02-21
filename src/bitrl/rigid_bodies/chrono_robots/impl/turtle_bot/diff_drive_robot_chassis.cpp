#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO


#include "chrono/assets/ChColor.h"

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/diff_drive_robot_chassis.h"

namespace bitrl{
namespace rb::bitrl_chrono
{

CHRONO_DiffDriveRobot_Chassis::CHRONO_DiffDriveRobot_Chassis(std::shared_ptr<chrono::ChContactMaterial> mat,
                                                             chrono::ChSystem* system,
                                                             const chrono::ChVector3d& body_pos,
                                                             const chrono::ChQuaternion<>& body_rot)
    :
    CHRONO_DiffDriveRobot_Part("chassis", false, mat, system, body_pos, body_rot, nullptr, true)
{}

void CHRONO_DiffDriveRobot_Chassis::init() {


    if (this -> is_initialized_)
    {
        return;
    }

    this -> do_init_("chassis", chrono::ChVector3d(0, 0, 0),
                      chrono::ChColor(0.4f, 0.4f, 0.7f), 100.0);

    const std::string vis_mesh_file = get_vis_mesh_file();
    auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(chrono::ChVector3d(0, 0, 0), chrono::ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    real_t mmass;
    chrono::ChVector3d mcog;
    chrono::ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    chrono::ChMatrix33<> principal_inertia_rot;
    chrono::ChVector3d principal_I;
    chrono::ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    body_->SetFrameCOMToRef(chrono::ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    body_->SetMass(mmass * density_);
    body_->SetInertiaXX(density_ * principal_I);
    body_->SetFrameRefToAbs(chrono::ChFrame<>(pos_, rot_));
    body_->SetFixed(fixed_);

    this -> add_collision_shapes();

    body_->GetCollisionModel()->SetFamily(static_cast<int_t>(CollisionFamily::CHASSIS));
    body_->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::ACTIVE_WHEEL));

    this -> add_visualization_assets();
    system_->Add(body_);
    this -> is_initialized_ = true;
}

void CHRONO_DiffDriveRobot_Chassis::translate(const chrono::ChVector3d& shift) {
    body_->SetPos(body_->GetPos() + shift);
}


}
}

#endif
