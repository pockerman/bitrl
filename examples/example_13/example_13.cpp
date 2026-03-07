#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"


#ifdef BITRL_LOG
#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#endif

#include "bitrl/bitrl_consts.h"
#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChBodyEasy.h>
#include "chrono/assets/ChVisualSystem.h"
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
// #include "chrono/assets/ChColorAsset.h"
// #include "chrono/assets/ChLineShape.h"


#include <filesystem>
#include <string>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/functions/ChFunctionConst.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

namespace
{
void draw_world_axes(chrono::irrlicht::ChVisualSystemIrrlicht& vis,
                     double scale = 1.0) {
    auto* driver = vis.GetVideoDriver();

    // X axis (red)
    driver->draw3DLine(
        {0, 0, 0},
        {static_cast<float>(scale), 0, 0},
        irr::video::SColor(255, 255, 0, 0));

    // Y axis (green)
    driver->draw3DLine(
        {0, 0, 0},
        {0, static_cast<float>(scale), 0},
        irr::video::SColor(255, 0, 255, 0));

    // Z axis (blue)
    driver->draw3DLine(
        {0, 0, 0},
        {0, 0, static_cast<float>(scale)},
        irr::video::SColor(255, 0, 0, 255));
}
}

int main() {

    // -----------------------------
    // Create Chrono physical system
    // -----------------------------
    ChSystemNSC system;
    //ChSystemSMC system;
    system.SetGravitationalAcceleration(chrono::ChVector3d(0,0,-9.81));
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    // -----------------------------
    // Create ground
    // -----------------------------
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();

    material->SetFriction(0.8f);
    material->SetRestitution(0.1f);
    auto floor = chrono_types::make_shared<chrono::ChBodyEasyBox>(
        5, 5, 0.1,     // size
        1000,          // density
        true, true, material);   // visual + collision

    floor->SetPos(chrono::ChVector3d(0,0,-0.05));
    floor->SetFixed(true);
    floor->EnableCollision(true);

    system.Add(floor);

    // -----------------------------
    // Robot chassis
    // -----------------------------
    double wheel_radius = 0.1;
    double chassis_height = 0.1;
    auto chassis = chrono_types::make_shared<ChBodyEasyBox>(
        0.5, 0.3, chassis_height,
        1000,
        true, true, material);

    // 0,0,wheel_radius + chassis_height/2.0
    //chrono::ChVector3d pos(0,0,0.2);
    chrono::ChVector3d pos(0.0, 0.0, wheel_radius + chassis_height/2.0 + 0.01);
    chassis->SetPos(pos);

    chrono::ChVector3d vel(0.5, 0.0, 0.0);
    chassis->SetPosDt(vel);
    system.Add(chassis);

    // -----------------------------
    // Wheels
    // -----------------------------

    double wheel_width = 0.05;
    //auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    auto left_wheel = chrono_types::make_shared<chrono::ChBodyEasyCylinder>(
        chrono::ChAxis::Y,
        wheel_radius,
        wheel_width,
        1000,
        true, true, material);

    auto right_wheel = chrono_types::make_shared<chrono::ChBodyEasyCylinder>(
    chrono::ChAxis::Y,
        wheel_radius,
        wheel_width,
        1000,
        true, true, material);

    left_wheel->SetPos(chrono::ChVector3d(0, 0.2, 0.1));
    right_wheel->SetPos(chrono::ChVector3d(0,-0.2, 0.1));

    system.Add(left_wheel);
    system.Add(right_wheel);

    // -----------------------------
    // Motors (differential drive)
    // -----------------------------
    auto motor_left = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
    auto motor_right = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();

    chrono::ChQuaternion<> rot = chrono::QuatFromAngleX(chrono::CH_PI_2);
    motor_left->Initialize(
        left_wheel,
        chassis,
        chrono::ChFrame<>(chrono::ChVector3d(0,0.2,0.1), rot)
    );

    motor_right->Initialize(
        right_wheel,
        chassis,
        chrono::ChFrame<>(chrono::ChVector3d(0,-0.2,0.1), rot)
    );

    system.Add(motor_left);
    system.Add(motor_right);

    // -----------------------------
    // Wheel speeds
    // -----------------------------
    auto speed_left = chrono_types::make_shared<chrono::ChFunctionConst>(-4.5);
    auto speed_right = chrono_types::make_shared<chrono::ChFunctionConst>(-4.5);

    motor_left->SetSpeedFunction(speed_left);
    motor_right->SetSpeedFunction(speed_right);

    double caster_radius = 0.05;
    auto caster = chrono_types::make_shared<ChBodyEasySphere>(
    caster_radius,       // radius
    1000,       // density
    true,       // visualization
    true,       // collision
    material);

    caster->SetPos(chrono::ChVector3d(0.2, 0, 0.05));
    caster->EnableCollision(true);

    system.Add(caster);

    auto caster_joint = chrono_types::make_shared<ChLinkLockRevolute>();

    caster_joint->Initialize(
        caster,
        chassis,
        ChFrame<>(chrono::ChVector3d(-0.25, 0, caster_radius), QUNIT));

    system.Add(caster_joint);

    floor->EnableCollision(true);
    chassis->EnableCollision(true);
    left_wheel->EnableCollision(true);
    right_wheel->EnableCollision(true);

    // -----------------------------
    // Irrlicht visualization
    // -----------------------------
    chrono::irrlicht::ChVisualSystemIrrlicht vis;

    vis.AttachSystem(&system);

    vis.SetWindowSize(1280,720);
    vis.SetWindowTitle("Chrono Differential Drive Robot");

    vis.Initialize();
    vis.AddSkyBox();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddCamera({0, -2, 1}, {0, 0, 0});
    vis.AddTypicalLights();
    vis.BindAll();
    // 1 meter length
    // -----------------------------
    // Simulation loop
    // -----------------------------
    double step_size = 0.002;

    while (vis.Run()) {

        vis.BeginScene();
        vis.Render();
        draw_world_axes(vis, 2.0);

        auto position = chassis -> GetPos();
        std::cout << position <<std::endl;

        system.DoStepDynamics(step_size);
        vis.EndScene();
    }

    return 0;
}
#else
#include <iostream>
int main()
{
    std::cerr<<"You need PROJECTCHRONO configured with "
             <<"bitrl in order to run this example "
             <<"Reconfigure bitrl and set ENABLE_CHRONO=ON"<<std::endl;
    return 1;
}

#endif