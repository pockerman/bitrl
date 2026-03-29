#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"
#include "bitrl/rigid_bodies/chrono_robots/turtle_robot.h"

#include <chrono/physics/ChBodyEasy.h>
#include <cmath>





namespace bitrl
{
namespace rb::bitrl_chrono{


// =============================================================================
// Create default contact material for the robot
std::shared_ptr<chrono::ChContactMaterial> build_default_contact_material(chrono::ChContactMethod contact_method) {
    float_t mu = 0.4f;   // coefficient of friction
    float_t cr = 0.0f;   // coefficient of restitution
    float_t Y = 2e7f;    // Young's modulus
    float_t nu = 0.3f;   // Poisson ratio
    float_t kn = 2e5f;   // normal stiffness
    float_t gn = 40.0f;  // normal viscous damping
    float_t kt = 2e5f;   // tangential stiffness
    float_t gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
    case chrono::ChContactMethod::NSC: {
        auto matNSC = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
        matNSC->SetFriction(mu);
        matNSC->SetRestitution(cr);
        return matNSC;
    }
    case chrono::ChContactMethod::SMC: {
        auto matSMC = chrono_types::make_shared<chrono::ChContactMaterialSMC>();
        matSMC->SetFriction(mu);
        matSMC->SetRestitution(cr);
        matSMC->SetYoungModulus(Y);
        matSMC->SetPoissonRatio(nu);
        matSMC->SetKn(kn);
        matSMC->SetGn(gn);
        matSMC->SetKt(kt);
        matSMC->SetGt(gt);
        return matSMC;
    }
    default:
        return std::shared_ptr<chrono::ChContactMaterial>();
    }
}

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
void add_revolute_joint(std::shared_ptr<chrono::ChBodyAuxRef> body_1,
                      std::shared_ptr<chrono::ChBodyAuxRef> body_2,
                      std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                      chrono::ChSystem* system,
                      const chrono::ChVector3d& rel_joint_pos,
                      const chrono::ChQuaternion<>& rel_joint_rot) {
    const chrono::ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    chrono::ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);         // parent -> child
    chrono::ChFrame<> X_GC = X_GP * X_PC;                         // global -> child

    auto revo = chrono_types::make_shared<chrono::ChLinkLockRevolute>();
    revo->Initialize(body_1, body_2, chrono::ChFrame<>(X_GC.GetCoordsys().pos, X_GC.GetCoordsys().rot));
    system->AddLink(revo);
}

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
void add_revolute_joint(std::shared_ptr<chrono::ChBodyEasyBox> body_1,
                      std::shared_ptr<chrono::ChBodyAuxRef> body_2,
                      std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                      chrono::ChSystem* system,
                      const chrono::ChVector3d& rel_joint_pos,
                      const chrono::ChQuaternion<>& rel_joint_rot) {
    const chrono::ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    chrono::ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);         // parent -> child
    chrono::ChFrame<> X_GC = X_GP * X_PC;                         // global -> child

    auto revo = chrono_types::make_shared<chrono::ChLinkLockRevolute>();
    revo->Initialize(body_1, body_2, chrono::ChFrame<>(X_GC.GetCoordsys().pos, X_GC.GetCoordsys().rot));
    system->AddLink(revo);
}

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
void add_lock_joint(std::shared_ptr<chrono::ChBodyAuxRef> body_1,
                  std::shared_ptr<chrono::ChBodyAuxRef> body_2,
                  std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                  chrono::ChSystem* system,
                  const chrono::ChVector3d& rel_joint_pos,
                  const chrono::ChQuaternion<>& rel_joint_rot) {
    const chrono::ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    chrono::ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);         // parent -> child
    chrono::ChFrame<> X_GC = X_GP * X_PC;                         // global -> child

    // auto revo = chrono_types::make_shared<ChLinkLockRevolute>();
    auto revo = chrono_types::make_shared<chrono::ChLinkLockLock>();
    revo->Initialize(body_1, body_2, chrono::ChFrame<>(X_GC.GetCoordsys().pos, X_GC.GetCoordsys().rot));
    system->AddLink(revo);
}

// Add a rotational speed controlled motor between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the motor
std::shared_ptr<chrono::ChLinkMotorRotationSpeed> add_motor(std::shared_ptr<chrono::ChBody> body_1,
                                                   std::shared_ptr<chrono::ChBodyAuxRef> body_2,
                                                   std::shared_ptr<chrono::ChBodyAuxRef> chassis,
                                                   chrono::ChSystem* system,
                                                   const chrono::ChVector3d& rel_joint_pos,
                                                   const chrono::ChQuaternion<>& rel_joint_rot,
                                                   std::shared_ptr<chrono::ChFunctionConst> speed_func) {
    const chrono::ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    chrono::ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);         // parent -> child
    chrono::ChFrame<> X_GC = X_GP * X_PC;                         // global -> child

    auto motor_angle = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
    motor_angle->Initialize(body_1, body_2, chrono::ChFrame<>(X_GC.GetCoordsys().pos, X_GC.GetCoordsys().rot));
    system->AddLink(motor_angle);
    motor_angle->SetSpeedFunction(speed_func);
    return motor_angle;
}


CHRONO_TurtleRobot::CHRONO_TurtleRobot(chrono::ChSystem& system,
                     const chrono::ChVector3d& robot_pos,
                     const chrono::ChQuaternion<>& robot_rot,
                     std::shared_ptr<chrono::ChContactMaterial> wheel_mat)
    :
    system_(&system),
    robot_pos_(robot_pos),
    robot_rot_(robot_rot),
    wheel_material_(wheel_mat),
    pose_()
    {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    auto contact_method = system_->GetContactMethod();
    if (contact_method == chrono::ChContactMethod::NSC) {
        chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    // Create the contact materials
    chassis_material_ = build_default_contact_material(contact_method);
    if (!wheel_material_)
        wheel_material_ = build_default_contact_material(contact_method);

}



void CHRONO_TurtleRobot::init() {
    // initialize robot chassis
    chassis_ = chrono_types::make_shared<CHRONO_DiffDriveRobot_Chassis>(chassis_material_, system_,
                                                                        robot_pos_, robot_rot_);
    chassis_ -> init();

    // set the pointer of the Chrono body that
    // the pose should trac
    pose_.set_body(chassis_->get_body_ptr());

    // active drive wheels' positions relative to the chassis
    real_t dwx = 0;
    real_t dwy = 0.11505;
    real_t dwz = 0.03735;
    drive_wheels_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_ActiveWheel>(
        "LDWheel", wheel_material_, system_, chrono::ChVector3d(dwx, +dwy, dwz), chrono::ChQuaternion<>(1, 0, 0, 0),
        chassis_->get_body_ptr()));
    drive_wheels_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_ActiveWheel>(
        "RDWheel", wheel_material_, system_, chrono::ChVector3d(dwx, -dwy, dwz), chrono::ChQuaternion<>(1, 0, 0, 0),
        chassis_->get_body_ptr()));

    // passive driven wheels' positions relative to the chassis
    real_t pwx = 0.11505;
    real_t pwy = 0;
    real_t pwz = 0.02015;

    passive_wheels_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_PassiveWheel>(
        "FPWheel",  wheel_material_, system_, chrono::ChVector3d(pwx, pwy, pwz), chrono::ChQuaternion<>(1, 0, 0, 0),
        chassis_->get_body_ptr()));
    passive_wheels_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_PassiveWheel>(
        "RPWheel", wheel_material_, system_, chrono::ChVector3d(-pwx, pwy, pwz), chrono::ChQuaternion<>(1, 0, 0, 0),
        chassis_->get_body_ptr()));

      for (int_t i = 0; i < 2; i++) {
        drive_wheels_[i]->init();
        passive_wheels_[i]->init();
    }

    // create the first level supporting rod
    real_t rod_s_0_x = -0.0565;
    real_t rod_s_0_y = 0.11992;
    real_t rod_s_0_z = 0.09615;

    real_t rod_s_1_x = 0.0535;
    real_t rod_s_1_y = 0.11992;
    real_t rod_s_1_z = 0.09615;

    real_t rod_s_2_x = 0.11850;
    real_t rod_s_2_y = 0.08192;
    real_t rod_s_2_z = 0.09615;

    real_t rod_s_3_x = 0.11850;
    real_t rod_s_3_y = -0.08192;
    real_t rod_s_3_z = 0.09615;

    real_t rod_s_4_x = 0.0535;
    real_t rod_s_4_y = -0.11992;
    real_t rod_s_4_z = 0.09615;

    real_t rod_s_5_x = -0.0565;
    real_t rod_s_5_y = -0.11992;
    real_t rod_s_5_z = 0.09615;

    m_1st_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "0-bottom-rod",  wheel_material_, system_, chrono::ChVector3d(rod_s_0_x, rod_s_0_y, rod_s_0_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_1st_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "1-bottom-rod",  wheel_material_, system_, chrono::ChVector3d(rod_s_1_x, rod_s_1_y, rod_s_1_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_1st_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "2-bottom-rod",  wheel_material_, system_, chrono::ChVector3d(rod_s_2_x, rod_s_2_y, rod_s_2_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_1st_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "3-bottom-rod",  wheel_material_, system_, chrono::ChVector3d(rod_s_3_x, rod_s_3_y, rod_s_3_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_1st_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "4-bottom-rod",  wheel_material_, system_, chrono::ChVector3d(rod_s_4_x, rod_s_4_y, rod_s_4_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_1st_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "5-bottom-rod",  wheel_material_, system_, chrono::ChVector3d(rod_s_5_x, rod_s_5_y, rod_s_5_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));

    // add the bottom plate
    real_t bt_plate_x = 0;
    real_t bt_plate_y = 0;
    real_t bt_plate_z = 0.14615;

    bottom_plate_ = chrono_types::make_shared<CHRONO_DiffDriveRobot_BottomPlate>(wheel_material_, system_,
        chrono::ChVector3d(bt_plate_x, bt_plate_y, bt_plate_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr());
    bottom_plate_ -> init();

    // create the second level support rod
    real_t rod_m_0_x = -0.10394;
    real_t rod_m_0_y = 0.09792;
    real_t rod_m_0_z = 0.15015;

    real_t rod_m_1_x = -0.0015;
    real_t rod_m_1_y = 0.16192;
    real_t rod_m_1_z = 0.15015;

    real_t rod_m_2_x = 0.0687;
    real_t rod_m_2_y = 0.13132;
    real_t rod_m_2_z = 0.15015;

    real_t rod_m_3_x = 0.0687;
    real_t rod_m_3_y = -0.13132;
    real_t rod_m_3_z = 0.15015;

    real_t rod_m_4_x = -0.0015;
    real_t rod_m_4_y = -0.16192;
    real_t rod_m_4_z = 0.15015;

    real_t rod_m_5_x = -0.10394;
    real_t rod_m_5_y = -0.09792;
    real_t rod_m_5_z = 0.15015;

    m_2nd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "0-middle-rod", wheel_material_, system_, chrono::ChVector3d(rod_m_0_x, rod_m_0_y, rod_m_0_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_2nd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "1-middle-rod", wheel_material_, system_, chrono::ChVector3d(rod_m_1_x, rod_m_1_y, rod_m_1_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_2nd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "2-middle-rod", wheel_material_, system_, chrono::ChVector3d(rod_m_2_x, rod_m_2_y, rod_m_2_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_2nd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "3-middle-rod", wheel_material_, system_, chrono::ChVector3d(rod_m_3_x, rod_m_3_y, rod_m_3_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_2nd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "4-middle-rod", wheel_material_, system_, chrono::ChVector3d(rod_m_4_x, rod_m_4_y, rod_m_4_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_2nd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "5-middle-rod", wheel_material_, system_, chrono::ChVector3d(rod_m_5_x, rod_m_5_y, rod_m_5_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));

    // add the middle plate
    real_t mi_plate_x = 0;
    real_t mi_plate_y = 0;
    real_t mi_plate_z = 0.20015;
    middle_plate_ = chrono_types::make_shared<CHRONO_DiffDriveRobot_MiddlePlate>(
        wheel_material_, system_, chrono::ChVector3d(mi_plate_x, mi_plate_y, mi_plate_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr());
    middle_plate_ -> init();

    // create the third level support rod
    real_t rod_u_0_x = -0.10394;
    real_t rod_u_0_y = 0.09792;
    real_t rod_u_0_z = 0.20615;

    real_t rod_u_1_x = -0.0015;
    real_t rod_u_1_y = 0.16192;
    real_t rod_u_1_z = 0.20615;

    real_t rod_u_2_x = 0.0687;
    real_t rod_u_2_y = 0.13132;
    real_t rod_u_2_z = 0.20615;

    real_t rod_u_3_x = 0.0687;
    real_t rod_u_3_y = -0.13132;
    real_t rod_u_3_z = 0.20615;

    real_t rod_u_4_x = -0.0015;
    real_t rod_u_4_y = -0.16192;
    real_t rod_u_4_z = 0.20615;

    real_t rod_u_5_x = -0.10394;
    real_t rod_u_5_y = -0.09792;
    real_t rod_u_5_z = 0.20615;

    m_3rd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "0-top-rod", wheel_material_, system_, chrono::ChVector3d(rod_u_0_x, rod_u_0_y, rod_u_0_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_3rd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "1-top-rod",  wheel_material_, system_, chrono::ChVector3d(rod_u_1_x, rod_u_1_y, rod_u_1_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_3rd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "2-top-rod", wheel_material_, system_, chrono::ChVector3d(rod_u_2_x, rod_u_2_y, rod_u_2_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_3rd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "3-top-rod", wheel_material_, system_, chrono::ChVector3d(rod_u_3_x, rod_u_3_y, rod_u_3_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_3rd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "4-top-rod", wheel_material_, system_, chrono::ChVector3d(rod_u_4_x, rod_u_4_y, rod_u_4_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));
    m_3rd_level_rods_.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "5-top-rod", wheel_material_, system_, chrono::ChVector3d(rod_u_5_x, rod_u_5_y, rod_u_5_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr()));


       for (int_t i = 0; i < 6; i++) {
        m_1st_level_rods_[i]->init();
        m_2nd_level_rods_[i]->init();
        m_3rd_level_rods_[i]->init();
    }

    // add the top plate
    real_t top_plate_x = 0;
    real_t top_plate_y = 0;
    real_t top_plate_z = 0.40615;
    top_plate_ = chrono_types::make_shared<CHRONO_DiffDriveRobot_TopPlate>(wheel_material_, system_,
                                                                chrono::ChVector3d(top_plate_x, top_plate_y, top_plate_z),
                                                                chrono::ChQuaternion<>(1, 0, 0, 0), chassis_->get_body_ptr());

    top_plate_ -> init();

    this -> build_motors_();
}

/// Initialize the complete rover and add all constraints
void CHRONO_TurtleRobot::build_motors_() {

    // redeclare necessary location variables
    real_t dwx = 0;
    real_t dwy = 0.11505;
    real_t dwz = 0.03735;
    real_t pwx = 0.11505;
    real_t pwy = 0;
    real_t pwz = 0.02015;
    real_t rod_s_0_x = -0.0565;
    real_t rod_s_0_y = 0.11992;
    real_t rod_s_0_z = 0.09615;
    real_t rod_s_1_x = 0.0535;
    real_t rod_s_1_y = 0.11992;
    real_t rod_s_1_z = 0.09615;
    real_t rod_s_2_x = 0.11850;
    real_t rod_s_2_y = 0.08192;
    real_t rod_s_2_z = 0.09615;
    real_t rod_s_3_x = 0.11850;
    real_t rod_s_3_y = -0.08192;
    real_t rod_s_3_z = 0.09615;
    real_t rod_s_4_x = 0.0535;
    real_t rod_s_4_y = -0.11992;
    real_t rod_s_4_z = 0.09615;
    real_t rod_s_5_x = -0.0565;
    real_t rod_s_5_y = -0.11992;
    real_t rod_s_5_z = 0.09615;
    real_t rod_m_0_x = -0.10394;
    real_t rod_m_0_y = 0.09792;
    real_t rod_m_0_z = 0.15015;
    real_t rod_m_1_x = -0.0015;
    real_t rod_m_1_y = 0.16192;
    real_t rod_m_1_z = 0.15015;
    real_t rod_m_2_x = 0.0687;
    real_t rod_m_2_y = 0.13132;
    real_t rod_m_2_z = 0.15015;
    real_t rod_m_3_x = 0.0687;
    real_t rod_m_3_y = -0.13132;
    real_t rod_m_3_z = 0.15015;
    real_t rod_m_4_x = -0.0015;
    real_t rod_m_4_y = -0.16192;
    real_t rod_m_4_z = 0.15015;
    real_t rod_m_5_x = -0.10394;
    real_t rod_m_5_y = -0.09792;
    real_t rod_m_5_z = 0.15015;
    real_t rod_u_0_x = -0.10394;
    real_t rod_u_0_y = 0.09792;
    real_t rod_u_0_z = 0.20615;
    real_t rod_u_1_x = -0.0015;
    real_t rod_u_1_y = 0.16192;
    real_t rod_u_1_z = 0.20615;
    real_t rod_u_2_x = 0.0687;
    real_t rod_u_2_y = 0.13132;
    real_t rod_u_2_z = 0.20615;
    real_t rod_u_3_x = 0.0687;
    real_t rod_u_3_y = -0.13132;
    real_t rod_u_3_z = 0.20615;
    real_t rod_u_4_x = -0.0015;
    real_t rod_u_4_y = -0.16192;
    real_t rod_u_4_z = 0.20615;
    real_t rod_u_5_x = -0.10394;
    real_t rod_u_5_y = -0.09792;
    real_t rod_u_5_z = 0.20615;

    // add motors and revolute joints on the active and passive wheels
    auto const_speed_function_l = chrono_types::make_shared<chrono::ChFunctionConst>(-chrono::CH_PI);
    auto const_speed_function_r = chrono_types::make_shared<chrono::ChFunctionConst>(-chrono::CH_PI);
    motors_func_.push_back(const_speed_function_l);
    motors_func_.push_back(const_speed_function_r);

    chrono::ChQuaternion<> z2y = chrono::QuatFromAngleX(chrono::CH_PI_2);
    chrono::ChQuaternion<> z2x = chrono::QuatFromAngleY(-chrono::CH_PI_2);

    motors_.push_back(add_motor(drive_wheels_[0]->get_body_ptr(), chassis_->get_body_ptr(), chassis_->get_body_ptr(), system_,
                                chrono::ChVector3d(dwx, dwy, dwz), z2y, const_speed_function_l));
    add_revolute_joint(passive_wheels_[0]->get_body_ptr(), chassis_->get_body_ptr(), chassis_->get_body_ptr(), system_,
                     chrono::ChVector3d(pwx, pwy, pwz), z2x);

    motors_.push_back(add_motor(drive_wheels_[1]->get_body_ptr(), chassis_->get_body_ptr(), chassis_->get_body_ptr(), system_,
                                chrono::ChVector3d(dwx, -dwy, dwz), z2y, const_speed_function_r));
    add_revolute_joint(passive_wheels_[1]->get_body_ptr(), chassis_->get_body_ptr(), chassis_->get_body_ptr(), system_,
                     chrono::ChVector3d(-pwx, pwy, pwz), z2x);

    // add fixity on all rods and plates
    // There are six constraints needed:
    // chassis -> bottom rods
    // bottom rods -> bottom plate
    // bottom plate -> middle rods
    // middle rods -> middle plate
    // middle plate -> top rods
    // top rods -> top plate

    chrono::ChVector3d bottom_rod_rel_pos[] = {chrono::ChVector3d(rod_s_0_x, rod_s_0_y, rod_s_0_z),  //
                                       chrono::ChVector3d(rod_s_1_x, rod_s_1_y, rod_s_1_z),  //
                                       chrono::ChVector3d(rod_s_2_x, rod_s_2_y, rod_s_2_z),  //
                                       chrono::ChVector3d(rod_s_3_x, rod_s_3_y, rod_s_3_z),  //
                                       chrono::ChVector3d(rod_s_4_x, rod_s_4_y, rod_s_4_z),  //
                                       chrono::ChVector3d(rod_s_5_x, rod_s_5_y, rod_s_5_z)};
    chrono::ChVector3d middle_rod_rel_pos[] = {chrono::ChVector3d(rod_m_0_x, rod_m_0_y, rod_m_0_z),  //
                                       chrono::ChVector3d(rod_m_1_x, rod_m_1_y, rod_m_1_z),  //
                                       chrono::ChVector3d(rod_m_2_x, rod_m_2_y, rod_m_2_z),  //
                                       chrono::ChVector3d(rod_m_3_x, rod_m_3_y, rod_m_3_z),  //
                                       chrono::ChVector3d(rod_m_4_x, rod_m_4_y, rod_m_4_z),  //
                                       chrono::ChVector3d(rod_m_5_x, rod_m_5_y, rod_m_5_z)};
    chrono::ChVector3d top_rod_rel_pos[] = {chrono::ChVector3d(rod_u_0_x, rod_u_0_y, rod_u_0_z),  //
                                    chrono::ChVector3d(rod_u_1_x, rod_u_1_y, rod_u_1_z),  //
                                    chrono::ChVector3d(rod_u_2_x, rod_u_2_y, rod_u_2_z),  //
                                    chrono::ChVector3d(rod_u_3_x, rod_u_3_y, rod_u_3_z),  //
                                    chrono::ChVector3d(rod_u_4_x, rod_u_4_y, rod_u_4_z),  //
                                    chrono::ChVector3d(rod_u_5_x, rod_u_5_y, rod_u_5_z)};

    for (int_t i = 0; i < 6; i++) {
        chrono::ChVector3d bottom_plate_rel_pos = bottom_rod_rel_pos[i] + chrono::ChVector3d(0, 0, 0.05);
        chrono::ChVector3d middle_plate_rel_pos = middle_rod_rel_pos[i] + chrono::ChVector3d(0, 0, 0.05);
        chrono::ChVector3d top_plate_rel_pos = top_rod_rel_pos[i] + chrono::ChVector3d(0, 0, 0.2);

        add_lock_joint(m_1st_level_rods_[i]->get_body_ptr(), chassis_->get_body_ptr(), chassis_->get_body_ptr(), system_,
                     bottom_rod_rel_pos[i], chrono::ChQuaternion<>(1, 0, 0, 0));
        add_lock_joint(m_1st_level_rods_[i]->get_body_ptr(), bottom_plate_->get_body_ptr(), chassis_->get_body_ptr(), system_,
                     bottom_plate_rel_pos, chrono::ChQuaternion<>(1, 0, 0, 0));
        add_lock_joint(m_2nd_level_rods_[i]->get_body_ptr(), bottom_plate_->get_body_ptr(), chassis_->get_body_ptr(), system_,
                     bottom_rod_rel_pos[i], chrono::ChQuaternion<>(1, 0, 0, 0));
        add_lock_joint(m_2nd_level_rods_[i]->get_body_ptr(), middle_plate_->get_body_ptr(), chassis_->get_body_ptr(), system_,
                     middle_plate_rel_pos, chrono::ChQuaternion<>(1, 0, 0, 0));
        add_lock_joint(m_3rd_level_rods_[i]->get_body_ptr(), middle_plate_->get_body_ptr(), chassis_->get_body_ptr(), system_,
                     top_rod_rel_pos[i], chrono::ChQuaternion<>(1, 0, 0, 0));
        add_lock_joint(m_3rd_level_rods_[i]->get_body_ptr(), top_plate_->get_body_ptr(), chassis_->get_body_ptr(), system_,
                     top_plate_rel_pos, chrono::ChQuaternion<>(1, 0, 0, 0));
    }
}

void CHRONO_TurtleRobot::set_motor_speed(real_t rad_speed, uint_t id) {
    motors_func_[id]->SetConstant(rad_speed);
}

chrono::ChVector3d CHRONO_TurtleRobot::get_active_wheel_speed(uint_t id)const {
    return drive_wheels_[id]->get_body_ptr()->GetPosDt();
}

chrono::ChVector3d CHRONO_TurtleRobot::get_active_wheel_angular_velocity(uint_t id)const {
    return drive_wheels_[id]->get_body_ptr()->GetAngVelParent();
}

}
}

#endif
