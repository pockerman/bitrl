#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/turtle_robot_rods.h"

namespace bitrl
{
namespace rb::bitrl_chrono
{

CHRONO_DiffDriveRobot_Rod_Short::CHRONO_DiffDriveRobot_Rod_Short(const std::string& name,

                                         std::shared_ptr<chrono::ChContactMaterial> mat,
                                         chrono::ChSystem* system,
                                         const chrono::ChVector3d& body_pos,
                                         const chrono::ChQuaternion<>& body_rot,
                                         std::shared_ptr<chrono::ChBodyAuxRef> chassis)
    :
CHRONO_DiffDriveRobot_Part(name, false, mat, system, body_pos, body_rot, chassis, true)
{
    // mesh_name_ = "support_rod_short";
    // offset_ = chrono::ChVector3d(0, 0, 0);
    // color_ = chrono::ChColor(0.4f, 0.4f, 0.7f);
    // density_ = 100;
}

void CHRONO_DiffDriveRobot_Rod_Short::init() {

    if (this -> is_initialized_)
    {
        return;
    }

    this -> do_init_("support_rod_short", chrono::ChVector3d(0, 0, 0),
                     chrono::ChColor(0.4f, 0.4f, 0.7f), 100.0);

    const std::string vis_mesh_file = get_vis_mesh_file();

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

    body_->SetFrameCOMToRef(chrono::ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    body_->SetMass(mmass * density_);
    body_->SetInertiaXX(density_ * principal_I);

    // set relative position to chassis
    const chrono::ChFrame<>& X_GP = chassis_->GetFrameRefToAbs();  // global -> parent
    chrono::ChFrame<> X_PC(pos_, rot_);                           // parent -> child
    chrono::ChFrame<> X_GC = X_GP * X_PC;                           // global -> child
    body_->SetFrameRefToAbs(X_GC);
    body_->SetFixed(fixed_);

    this->add_collision_shapes();

    body_->GetCollisionModel()->SetFamily(static_cast<int_t>(CollisionFamily::ROD));
    body_->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::CHASSIS));
    body_->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::BOTTOM_PLATE));
    body_->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::MIDDLE_PLATE));

    this->add_visualization_assets();
    system_->Add(body_);
    this -> is_initialized_ = true;
}



void CHRONO_DiffDriveRobot_Rod_Short::translate(const chrono::ChVector3d& shift) {
    body_->SetPos(body_->GetPos() + shift);
}

CHRONO_DiffDriveRobot_Rod_Long::CHRONO_DiffDriveRobot_Rod_Long(const std::string& name,

                                       std::shared_ptr<chrono::ChContactMaterial> mat,
                                       chrono::ChSystem* system,
                                       const chrono::ChVector3d& body_pos,
                                       const chrono::ChQuaternion<>& body_rot,
                                       std::shared_ptr<chrono::ChBodyAuxRef> chassis)
    :
    CHRONO_DiffDriveRobot_Part(name, false, mat, system, body_pos, body_rot, chassis, true)
{
    // mesh_name_ = "support_rod_long";
    // offset_ = chrono::ChVector3d(0, 0, 0);
    // color_ = chrono::ChColor(0.4f, 0.4f, 0.7f);
    // density_ = 100;
}

void CHRONO_DiffDriveRobot_Rod_Long::init() {

    if (this -> is_initialized_)
    {
        return;
    }

    this -> do_init_("support_rod_long", chrono::ChVector3d(0, 0, 0),
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

    // set relative position to chassis
    const chrono::ChFrame<>& X_GP = chassis_->GetFrameRefToAbs();  // global -> parent
    chrono::ChFrame<> X_PC(pos_, rot_);                           // parent -> child
    chrono::ChFrame<> X_GC = X_GP * X_PC;                           // global -> child
    body_->SetFrameRefToAbs(X_GC);
    body_->SetFixed(fixed_);

    this->add_collision_shapes();

    body_->GetCollisionModel()->SetFamily(static_cast<int_t>(CollisionFamily::ROD));
    body_->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::CHASSIS));
    body_->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::BOTTOM_PLATE));
    body_->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::MIDDLE_PLATE));
    body_->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::ROD));

    this->add_visualization_assets();

    system_->Add(body_);
    this -> is_initialized_ = true;
}

void CHRONO_DiffDriveRobot_Rod_Long::translate(const chrono::ChVector3d& shift) {
    body_->SetPos(body_->GetPos() + shift);
}


}
}
#endif
