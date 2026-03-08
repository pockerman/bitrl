#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"

#ifdef BITRL_LOG
#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#endif

#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChBodyEasy.h>
#include "chrono/assets/ChVisualSystem.h"
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <chrono/physics/ChLinkMotorRotationSpeed.h>
#include <chrono/functions/ChFunctionConst.h>

#include <memory>

namespace example_12
{

using namespace chrono;
using namespace chrono::irrlicht;
using namespace bitrl;

const real_t WHEEL_RADIUS = 0.1;
const real_t CHASSIS_HEIGHT = 0.1;
const real_t WHEEL_WIDTH = 0.05;
const real_t CASTER_RADIUS = 0.05;

void draw_world_axes(chrono::irrlicht::ChVisualSystemIrrlicht& vis,
                     real_t scale = 1.0) {
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

auto build_generic_material()
{
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();
    material->SetFriction(0.8f);
    material->SetRestitution(0.1f);
    return material;
}

auto build_floor(std::shared_ptr<ChContactMaterialNSC> material)
{
    auto floor = chrono_types::make_shared<chrono::ChBodyEasyBox>(
         5, 5, 0.1,     // size
         1000,          // density
         true, true, material);   // visual + collision

    floor->SetPos(chrono::ChVector3d(0,0,-0.05));
    floor->SetFixed(true);
    floor->EnableCollision(true);
    return floor;
}

auto build_chassis(std::shared_ptr<ChContactMaterialNSC> material)
{
    auto chassis = chrono_types::make_shared<ChBodyEasyBox>(
        0.5, 0.3, CHASSIS_HEIGHT,
        1000,
        true, true, material);

    chrono::ChVector3d pos(0.0, 0.0, WHEEL_RADIUS + CHASSIS_HEIGHT/2.0 + 0.01);
    chassis->SetPos(pos);

    chrono::ChVector3d vel(0.5, 0.0, 0.0);
    chassis->SetPosDt(vel);
    chassis->EnableCollision(true);
    return chassis;
}

auto build_wheel(std::shared_ptr<ChContactMaterialNSC> material, const chrono::ChVector3d& pos)
{
    auto wheel = chrono_types::make_shared<chrono::ChBodyEasyCylinder>(
          chrono::ChAxis::Y,
          WHEEL_RADIUS,
          WHEEL_WIDTH,
          1000,
          true, true, material);

    wheel->SetPos(pos);
    wheel->EnableCollision(true);
    return wheel;
}

auto build_motor(std::shared_ptr<ChBodyEasyCylinder> wheel,
                 std::shared_ptr<ChBodyEasyBox> chassis,
                 std::shared_ptr<ChFunctionConst> speed_func,
                 const chrono::ChVector3d& pos)
{
    auto motor = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();

    chrono::ChQuaternion<> rot = chrono::QuatFromAngleX(chrono::CH_PI_2);
    motor->Initialize(
        wheel,
        chassis,
        chrono::ChFrame<>(pos, rot)
    );

    motor->SetSpeedFunction(speed_func);
    return motor;
}

auto build_caster_wheel(std::shared_ptr<ChContactMaterialNSC> material)
{

    auto caster = chrono_types::make_shared<ChBodyEasySphere>(
    CASTER_RADIUS,       // radius
    1000,       // density
    true,       // visualization
    true,       // collision
    material);

    caster->SetPos(chrono::ChVector3d(0.2, 0, 0.05));
    caster->EnableCollision(true);
    return caster;
}

auto build_caster_joint(std::shared_ptr<ChBodyEasySphere> caster,
                        std::shared_ptr<ChBodyEasyBox> chassis)
{
    auto caster_joint = chrono_types::make_shared<ChLinkLockRevolute>();

    caster_joint->Initialize(
        caster,
        chassis,
        ChFrame<>(chrono::ChVector3d(-0.25, 0, CASTER_RADIUS), QUNIT));
    return caster_joint;

}
}

int main() {

    using namespace example_12;
    ChSystemNSC system;
    system.SetGravitationalAcceleration(chrono::ChVector3d(0,0,-bitrl::consts::maths::G));
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    // create a generic material to use
    auto material = example_12::build_generic_material();

    // build floor
    auto floor = example_12::build_floor(material);
    system.Add(floor);

    // build chassis
    auto chassis = example_12::build_chassis(material);
    system.Add(chassis);

    // build wheels
    auto left_wheel = example_12::build_wheel(material,chrono::ChVector3d(0, 0.2, 0.1));
    auto right_wheel = example_12::build_wheel(material,chrono::ChVector3d(0,-0.2, 0.1));
    system.Add(left_wheel);
    system.Add(right_wheel);

    // build motor
    auto motor_left = example_12::build_motor(left_wheel, chassis,
        chrono_types::make_shared<chrono::ChFunctionConst>(-4.5), chrono::ChVector3d(0,0.2,0.1));
    auto motor_right = build_motor(right_wheel, chassis,
        chrono_types::make_shared<chrono::ChFunctionConst>(-4.5), chrono::ChVector3d(0,-0.2,0.1));

    system.Add(motor_left);
    system.Add(motor_right);

    auto caster = example_12::build_caster_wheel(material);
    system.Add(caster);

    auto caster_joint = example_12::build_caster_joint(caster, chassis);
    system.Add(caster_joint);

    // create the visualization system
    chrono::irrlicht::ChVisualSystemIrrlicht vis;

    vis.AttachSystem(&system);
    vis.SetWindowSize(1280,720);
    vis.SetWindowTitle("Example 12 Differential Drive Robot");
    vis.Initialize();
    vis.AddSkyBox();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddCamera({0, -2, 1}, {0, 0, 0});
    vis.AddTypicalLights();
    vis.BindAll();

    real_t step_size = 0.002;
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