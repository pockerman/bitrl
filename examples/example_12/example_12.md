\page bitrl_example_12 BitRL Example 12 A rigid body simulation with Chrono

Example \ref bitrl_example_11 discussed _ChBody_. Specifically, how to create a _ChBodyEasyBox_.
Understanding rigid bodies is fundamental to robotics and further examples in this series will dive deeper into
this subject. In this example we want to create a simple simulation of a <a href="https://en.wikipedia.org/wiki/Differential_wheeled_robot">differential drive robot</a>.
According to wikipedia:

_A differential wheeled robot is a mobile robot whose movement is based on two separately 
driven wheels placed on either side of the robot body. It can thus change its direction by varying the 
relative rate of rotation of its wheels and hence does not require an additional steering motion. 
Robots with such a drive typically have one or more caster wheels to prevent the vehicle from tilting._

Thus, we will build a robot with two motorised wheels and one passive caster wheel useful for balancing the robot.

Running a simulation with Chrono requires that we create a _ChSystem_ instance 
(see <a href="https://api.projectchrono.org/9.0.0/simulation_system.html">Simulation system</a> for more information).
A _ChSystem_ is an abstract class. The Chrono library provides the following subclasses:

- _ChSystemNSC_ for Non Smooth Contacts (NSC): in case of contacts a complementarity solver will take care of them using non-smooth dynamics; this is very efficient even with large time steps.
- _ChSystemSMC_ for Smooth Contacts (SMC): contacts are handled using penalty methods, i.e. contacts are deformable.

Note that if there are no contacts or collisions in your system, it is indifferent to use _ChSystemNSC_ or _ChSystemSMC_.

We will use the _ChSystemNSC_ class in the example below as the wheels of the robot will be in contact with the ground.
The code below evolves around a number of helper functions that create the various components we will be adding to
the _ChSystemSMC_. A lot of things are hardcoded i.e. not necessarily the best way we want to do things.
However, in this tutorial we want to keep things simple and concentrate more on the essential elements of a Chrono simulation.

The include files

@code{.cpp}
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
@endcode

Next are the helper functions that set up the various components. Note that we need to enable collisions.
We also need to set explicitly the velocity of the chassis explicitly. The linear velocity of a differential drive system is given by

$$
v = \frac{r}{2}(\omega_R + \omega_L)
$$

where \f$r\f$ is the radius of the wheel and \f$\omega_i\f$ if the angular velocity of motor \f$i\f$ in rad/s.
We can use this relationship in a control simulation in order to control either the linear velocity or the motor speed.

@code{.cpp}
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
@endcode

Below is the main driver


@code{.cpp}
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
@endcode