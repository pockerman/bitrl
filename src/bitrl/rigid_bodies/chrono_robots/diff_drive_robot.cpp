#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"

#include <cmath>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/physics/ChMassProperties.h"



namespace bitrl
{
namespace rb::bitrl_chrono{


// =============================================================================
// Create default contact material for the robot
std::shared_ptr<chrono::ChContactMaterial> DefaultContactMaterial(chrono::ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.0f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

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
void AddRevoluteJoint(std::shared_ptr<chrono::ChBodyAuxRef> body_1,
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
void AddRevoluteJoint(std::shared_ptr<chrono::ChBodyEasyBox> body_1,
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
void AddLockJoint(std::shared_ptr<chrono::ChBodyAuxRef> body_1,
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
std::shared_ptr<chrono::ChLinkMotorRotationSpeed> AddMotor(std::shared_ptr<chrono::ChBody> body_1,
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


CHRONO_DiffDriveRobot::CHRONO_DiffDriveRobot(chrono::ChSystem& system,
                     const chrono::ChVector3d& robot_pos,
                     const chrono::ChQuaternion<>& robot_rot,
                     std::shared_ptr<chrono::ChContactMaterial> wheel_mat)
    : m_system(&system), m_robot_pos(robot_pos), m_robot_rot(robot_rot), m_wheel_material(wheel_mat) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    auto contact_method = m_system->GetContactMethod();
    if (contact_method == chrono::ChContactMethod::NSC) {
        chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    // Create the contact materials
    m_chassis_material = DefaultContactMaterial(contact_method);
    if (!m_wheel_material)
        m_wheel_material = DefaultContactMaterial(contact_method);

    create();
}



void CHRONO_DiffDriveRobot::create() {
    // initialize robot chassis
    m_chassis = chrono_types::make_shared<CHRONO_DiffDriveRobot_Chassis>("chassis", false, m_chassis_material, m_system,
                                                             m_robot_pos, m_robot_rot, true);

    // active drive wheels' positions relative to the chassis
    double dwx = 0;
    double dwy = 0.11505;
    double dwz = 0.03735;
    m_drive_wheels.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_ActiveWheel>(
        "LDWheel", false, m_wheel_material, m_system, chrono::ChVector3d(dwx, +dwy, dwz), chrono::ChQuaternion<>(1, 0, 0, 0),
        m_chassis->get_body_ptr(), true));
    m_drive_wheels.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_ActiveWheel>(
        "RDWheel", false, m_wheel_material, m_system, chrono::ChVector3d(dwx, -dwy, dwz), chrono::ChQuaternion<>(1, 0, 0, 0),
        m_chassis->get_body_ptr(), true));

    // passive driven wheels' positions relative to the chassis
    double pwx = 0.11505;
    double pwy = 0;
    double pwz = 0.02015;

    m_passive_wheels.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_PassiveWheel>(
        "FPWheel", false, m_wheel_material, m_system, chrono::ChVector3d(pwx, pwy, pwz), chrono::ChQuaternion<>(1, 0, 0, 0),
        m_chassis->get_body_ptr(), true));
    m_passive_wheels.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_PassiveWheel>(
        "RPWheel", false, m_wheel_material, m_system, chrono::ChVector3d(-pwx, pwy, pwz), chrono::ChQuaternion<>(1, 0, 0, 0),
        m_chassis->get_body_ptr(), true));

    // create the first level supporting rod
    double rod_s_0_x = -0.0565;
    double rod_s_0_y = 0.11992;
    double rod_s_0_z = 0.09615;

    double rod_s_1_x = 0.0535;
    double rod_s_1_y = 0.11992;
    double rod_s_1_z = 0.09615;

    double rod_s_2_x = 0.11850;
    double rod_s_2_y = 0.08192;
    double rod_s_2_z = 0.09615;

    double rod_s_3_x = 0.11850;
    double rod_s_3_y = -0.08192;
    double rod_s_3_z = 0.09615;

    double rod_s_4_x = 0.0535;
    double rod_s_4_y = -0.11992;
    double rod_s_4_z = 0.09615;

    double rod_s_5_x = -0.0565;
    double rod_s_5_y = -0.11992;
    double rod_s_5_z = 0.09615;

    m_1st_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "0-bottom-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_s_0_x, rod_s_0_y, rod_s_0_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_1st_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "1-bottom-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_s_1_x, rod_s_1_y, rod_s_1_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_1st_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "2-bottom-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_s_2_x, rod_s_2_y, rod_s_2_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_1st_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "3-bottom-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_s_3_x, rod_s_3_y, rod_s_3_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_1st_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "4-bottom-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_s_4_x, rod_s_4_y, rod_s_4_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_1st_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "5-bottom-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_s_5_x, rod_s_5_y, rod_s_5_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));

    // add the bottom plate
    double bt_plate_x = 0;
    double bt_plate_y = 0;
    double bt_plate_z = 0.14615;

    m_bottom_plate = chrono_types::make_shared<CHRONO_DiffDriveRobot_BottomPlate>(
        "bottom_plate", false, m_wheel_material, m_system, chrono::ChVector3d(bt_plate_x, bt_plate_y, bt_plate_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true);

    // create the second level support rod
    double rod_m_0_x = -0.10394;
    double rod_m_0_y = 0.09792;
    double rod_m_0_z = 0.15015;

    double rod_m_1_x = -0.0015;
    double rod_m_1_y = 0.16192;
    double rod_m_1_z = 0.15015;

    double rod_m_2_x = 0.0687;
    double rod_m_2_y = 0.13132;
    double rod_m_2_z = 0.15015;

    double rod_m_3_x = 0.0687;
    double rod_m_3_y = -0.13132;
    double rod_m_3_z = 0.15015;

    double rod_m_4_x = -0.0015;
    double rod_m_4_y = -0.16192;
    double rod_m_4_z = 0.15015;

    double rod_m_5_x = -0.10394;
    double rod_m_5_y = -0.09792;
    double rod_m_5_z = 0.15015;

    m_2nd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "0-middle-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_m_0_x, rod_m_0_y, rod_m_0_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_2nd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "1-middle-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_m_1_x, rod_m_1_y, rod_m_1_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_2nd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "2-middle-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_m_2_x, rod_m_2_y, rod_m_2_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_2nd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "3-middle-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_m_3_x, rod_m_3_y, rod_m_3_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_2nd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "4-middle-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_m_4_x, rod_m_4_y, rod_m_4_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_2nd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Short>(
        "5-middle-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_m_5_x, rod_m_5_y, rod_m_5_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));

    // add the middle plate
    double mi_plate_x = 0;
    double mi_plate_y = 0;
    double mi_plate_z = 0.20015;
    m_middle_plate = chrono_types::make_shared<CHRONO_DiffDriveRobot_MiddlePlate>(
        "middle_plate", false, m_wheel_material, m_system, chrono::ChVector3d(mi_plate_x, mi_plate_y, mi_plate_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true);

    // create the third level support rod
    double rod_u_0_x = -0.10394;
    double rod_u_0_y = 0.09792;
    double rod_u_0_z = 0.20615;

    double rod_u_1_x = -0.0015;
    double rod_u_1_y = 0.16192;
    double rod_u_1_z = 0.20615;

    double rod_u_2_x = 0.0687;
    double rod_u_2_y = 0.13132;
    double rod_u_2_z = 0.20615;

    double rod_u_3_x = 0.0687;
    double rod_u_3_y = -0.13132;
    double rod_u_3_z = 0.20615;

    double rod_u_4_x = -0.0015;
    double rod_u_4_y = -0.16192;
    double rod_u_4_z = 0.20615;

    double rod_u_5_x = -0.10394;
    double rod_u_5_y = -0.09792;
    double rod_u_5_z = 0.20615;

    m_3rd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "0-top-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_u_0_x, rod_u_0_y, rod_u_0_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_3rd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "1-top-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_u_1_x, rod_u_1_y, rod_u_1_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_3rd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "2-top-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_u_2_x, rod_u_2_y, rod_u_2_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_3rd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "3-top-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_u_3_x, rod_u_3_y, rod_u_3_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_3rd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "4-top-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_u_4_x, rod_u_4_y, rod_u_4_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));
    m_3rd_level_rods.push_back(chrono_types::make_shared<CHRONO_DiffDriveRobot_Rod_Long>(
        "5-top-rod", false, m_wheel_material, m_system, chrono::ChVector3d(rod_u_5_x, rod_u_5_y, rod_u_5_z),
        chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true));

    // add the top plate
    double top_plate_x = 0;
    double top_plate_y = 0;
    double top_plate_z = 0.40615;
    m_top_plate = chrono_types::make_shared<CHRONO_DiffDriveRobot_TopPlate>("top_plate", false, m_wheel_material, m_system,
                                                                chrono::ChVector3d(top_plate_x, top_plate_y, top_plate_z),
                                                                chrono::ChQuaternion<>(1, 0, 0, 0), m_chassis->get_body_ptr(), true);
}

/// Initialize the complete rover and add all constraints
void CHRONO_DiffDriveRobot::init() {

    m_chassis->init();
    m_bottom_plate->init();
    m_middle_plate->init();
    m_top_plate->init();
    for (int i = 0; i < 2; i++) {
        m_drive_wheels[i]->init();
        m_passive_wheels[i]->init();
    }

    for (int i = 0; i < 6; i++) {
        m_1st_level_rods[i]->init();
        m_2nd_level_rods[i]->init();
        m_3rd_level_rods[i]->init();
    }

    // redeclare necessary location variables
    double dwx = 0;
    double dwy = 0.11505;
    double dwz = 0.03735;

    double pwx = 0.11505;
    double pwy = 0;
    double pwz = 0.02015;

    double rod_s_0_x = -0.0565;
    double rod_s_0_y = 0.11992;
    double rod_s_0_z = 0.09615;

    double rod_s_1_x = 0.0535;
    double rod_s_1_y = 0.11992;
    double rod_s_1_z = 0.09615;

    double rod_s_2_x = 0.11850;
    double rod_s_2_y = 0.08192;
    double rod_s_2_z = 0.09615;

    double rod_s_3_x = 0.11850;
    double rod_s_3_y = -0.08192;
    double rod_s_3_z = 0.09615;

    double rod_s_4_x = 0.0535;
    double rod_s_4_y = -0.11992;
    double rod_s_4_z = 0.09615;

    double rod_s_5_x = -0.0565;
    double rod_s_5_y = -0.11992;
    double rod_s_5_z = 0.09615;

    double rod_m_0_x = -0.10394;
    double rod_m_0_y = 0.09792;
    double rod_m_0_z = 0.15015;

    double rod_m_1_x = -0.0015;
    double rod_m_1_y = 0.16192;
    double rod_m_1_z = 0.15015;

    double rod_m_2_x = 0.0687;
    double rod_m_2_y = 0.13132;
    double rod_m_2_z = 0.15015;

    double rod_m_3_x = 0.0687;
    double rod_m_3_y = -0.13132;
    double rod_m_3_z = 0.15015;

    double rod_m_4_x = -0.0015;
    double rod_m_4_y = -0.16192;
    double rod_m_4_z = 0.15015;

    double rod_m_5_x = -0.10394;
    double rod_m_5_y = -0.09792;
    double rod_m_5_z = 0.15015;

    double rod_u_0_x = -0.10394;
    double rod_u_0_y = 0.09792;
    double rod_u_0_z = 0.20615;

    double rod_u_1_x = -0.0015;
    double rod_u_1_y = 0.16192;
    double rod_u_1_z = 0.20615;

    double rod_u_2_x = 0.0687;
    double rod_u_2_y = 0.13132;
    double rod_u_2_z = 0.20615;

    double rod_u_3_x = 0.0687;
    double rod_u_3_y = -0.13132;
    double rod_u_3_z = 0.20615;

    double rod_u_4_x = -0.0015;
    double rod_u_4_y = -0.16192;
    double rod_u_4_z = 0.20615;

    double rod_u_5_x = -0.10394;
    double rod_u_5_y = -0.09792;
    double rod_u_5_z = 0.20615;

    // add motors and revolute joints on the active and passive wheels
    auto const_speed_function_l = chrono_types::make_shared<chrono::ChFunctionConst>(-chrono::CH_PI);
    auto const_speed_function_r = chrono_types::make_shared<chrono::ChFunctionConst>(-chrono::CH_PI);
    m_motors_func.push_back(const_speed_function_l);
    m_motors_func.push_back(const_speed_function_r);

    chrono::ChQuaternion<> z2y = chrono::QuatFromAngleX(chrono::CH_PI_2);
    chrono::ChQuaternion<> z2x = chrono::QuatFromAngleY(-chrono::CH_PI_2);

    m_motors.push_back(AddMotor(m_drive_wheels[0]->get_body_ptr(), m_chassis->get_body_ptr(), m_chassis->get_body_ptr(), m_system,
                                chrono::ChVector3d(dwx, dwy, dwz), z2y, const_speed_function_l));
    AddRevoluteJoint(m_passive_wheels[0]->get_body_ptr(), m_chassis->get_body_ptr(), m_chassis->get_body_ptr(), m_system,
                     chrono::ChVector3d(pwx, pwy, pwz), z2x);

    m_motors.push_back(AddMotor(m_drive_wheels[1]->get_body_ptr(), m_chassis->get_body_ptr(), m_chassis->get_body_ptr(), m_system,
                                chrono::ChVector3d(dwx, -dwy, dwz), z2y, const_speed_function_r));
    AddRevoluteJoint(m_passive_wheels[1]->get_body_ptr(), m_chassis->get_body_ptr(), m_chassis->get_body_ptr(), m_system,
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

    for (int i = 0; i < 6; i++) {
        chrono::ChVector3d bottom_plate_rel_pos = bottom_rod_rel_pos[i] + chrono::ChVector3d(0, 0, 0.05);
        chrono::ChVector3d middle_plate_rel_pos = middle_rod_rel_pos[i] + chrono::ChVector3d(0, 0, 0.05);
        chrono::ChVector3d top_plate_rel_pos = top_rod_rel_pos[i] + chrono::ChVector3d(0, 0, 0.2);

        AddLockJoint(m_1st_level_rods[i]->get_body_ptr(), m_chassis->get_body_ptr(), m_chassis->get_body_ptr(), m_system,
                     bottom_rod_rel_pos[i], chrono::ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_1st_level_rods[i]->get_body_ptr(), m_bottom_plate->get_body_ptr(), m_chassis->get_body_ptr(), m_system,
                     bottom_plate_rel_pos, chrono::ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_2nd_level_rods[i]->get_body_ptr(), m_bottom_plate->get_body_ptr(), m_chassis->get_body_ptr(), m_system,
                     bottom_rod_rel_pos[i], chrono::ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_2nd_level_rods[i]->get_body_ptr(), m_middle_plate->get_body_ptr(), m_chassis->get_body_ptr(), m_system,
                     middle_plate_rel_pos, chrono::ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_3rd_level_rods[i]->get_body_ptr(), m_middle_plate->get_body_ptr(), m_chassis->get_body_ptr(), m_system,
                     top_rod_rel_pos[i], chrono::ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_3rd_level_rods[i]->get_body_ptr(), m_top_plate->get_body_ptr(), m_chassis->get_body_ptr(), m_system,
                     top_plate_rel_pos, chrono::ChQuaternion<>(1, 0, 0, 0));
    }
}

void CHRONO_DiffDriveRobot::set_motor_speed(real_t rad_speed, uint_t id) {
    m_motors_func[id]->SetConstant(rad_speed);
}

chrono::ChVector3d CHRONO_DiffDriveRobot::get_active_wheel_speed(uint_t id)const {
    return m_drive_wheels[id]->get_body_ptr()->GetPosDt();
}

chrono::ChVector3d CHRONO_DiffDriveRobot::get_active_wheel_angular_velocity(uint_t id)const {
    return m_drive_wheels[id]->get_body_ptr()->GetAngVelParent();
}

}
}

#endif
