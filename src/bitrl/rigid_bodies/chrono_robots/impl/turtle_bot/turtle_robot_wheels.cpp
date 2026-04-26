#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"
#include "bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/turtle_robot_wheels.h"
#include "chrono/assets/ChColor.h"

#include "bitrl/bitrl_consts.h"


namespace bitrl
{
namespace rb::bitrl_chrono
{

CHRONO_DiffDriveRobot_ActiveWheel::CHRONO_DiffDriveRobot_ActiveWheel(const std::string& name,
                                             std::shared_ptr<chrono::ChContactMaterial> mat,
                                             chrono::ChSystem* system,
                                             const chrono::ChVector3d& body_pos,
                                             const chrono::ChQuaternion<>& body_rot,
                                             std::shared_ptr<chrono::ChBodyAuxRef> chassis)
    :
    CHRONO_DiffDriveRobot_Part(name, true, mat, system, body_pos, body_rot, chassis, false)
{
    // mesh_name_ = "active_wheel";
    // offset_ = chrono::ChVector3d(0, 0, 0);
    // color_ = chrono::ChColor(0.4f, 0.4f, 0.7f);
    // density_ = 200;
}

void CHRONO_DiffDriveRobot_ActiveWheel::init() {


    if (this -> is_initialized_)
    {
        return;
    }

    this -> do_init_("active_wheel", chrono::ChVector3d(0, 0, 0),
                     chrono::ChColor(0.4f, 0.4f, 0.7f), 200.0);

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

    this -> add_collision_shapes();

    body_->GetCollisionModel()->SetFamily(static_cast<int_t>(CollisionFamily::ACTIVE_WHEEL));
    body_->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::CHASSIS));

    this -> add_visualization_assets();

    system_->Add(body_);
    this -> is_initialized_ = true;
}

void CHRONO_DiffDriveRobot_ActiveWheel::translate(const chrono::ChVector3d& shift) {
    body_->SetPos(body_->GetPos() + shift);
}


CHRONO_DiffDriveRobot_PassiveWheel::CHRONO_DiffDriveRobot_PassiveWheel(const std::string& name,
                                               std::shared_ptr<chrono::ChContactMaterial> mat,
                                               chrono::ChSystem* system,
                                               const chrono::ChVector3d& body_pos,
                                               const chrono::ChQuaternion<>& body_rot,
                                               std::shared_ptr<chrono::ChBodyAuxRef> chassis
                                               )
    :
    CHRONO_DiffDriveRobot_Part(name, false, mat, system, body_pos, body_rot, chassis, true) {
    // mesh_name_ = "passive_wheel";
    // offset_ = chrono::ChVector3d(0, 0, 0);
    // color_ = chrono::ChColor(0.4f, 0.4f, 0.7f);
    // density_ = 200;
}

void CHRONO_DiffDriveRobot_PassiveWheel::init() {

    if (this -> is_initialized_)
    {
        return;
    }

    this -> do_init_("passive_wheel", chrono::ChVector3d(0, 0, 0),
                     chrono::ChColor(0.4f, 0.4f, 0.7f), 200.0);

    //const std::string vis_mesh_file(consts::ROBOTS_DIR + "/diff_drive_robot/" + mesh_name_ + ".obj");
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

    this -> add_collision_shapes();

    body_->GetCollisionModel()->SetFamily(static_cast<int_t>(CollisionFamily::PASSIVE_WHEEL));
    body_->GetCollisionModel()->DisallowCollisionsWith(static_cast<int_t>(CollisionFamily::CHASSIS));

    this -> add_visualization_assets();

    system_->Add(body_);
    this -> is_initialized_ = true;
}

void CHRONO_DiffDriveRobot_PassiveWheel::translate(const chrono::ChVector3d& shift) {
    body_->SetPos(body_->GetPos() + shift);
}

}
}
#endif