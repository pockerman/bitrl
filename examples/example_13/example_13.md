\page bitrl_example_13 BitRL Example 13 Using chrono::ChSystemSMC

Example \ref bitrl_example_12 showed you how to create a rigid body simulation in Chrono. We will extend this example
and show how to create a reinforcement learning environment that can be queried by reinforcement learning agents.

Reinforcement learning in _bitrl_ should implement the \ref bitrl::envs::EnvBase "`bitrl::envs::EnvBase`" class.
This is an abstract base class that defines the main endpoints that a reinforcement learning environment should implement.
We will keep things simple in this example and assume that the action space is discrete with three actions available.
We won't be doing anything meaningful in this example other than showing how a reinforcement learning environment can be 
implemented using Chrono. 

The state space is equally simple and returns the position of the robot. 
Below is the driver implementing this. To  a large extent this is the same as example \ref bitrl_example_12.
We have also introduced the headers _robot.h_ and _robot_env.h_ to separate concerns.

@code{.cpp}
#include "bitrl/bitrl_config.h"

#if defined(BITRL_CHRONO) && defined(BITRL_IRRLICHT)

#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/render/irrlicht_utils.h"

#ifdef BITRL_LOG
#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#endif

#include "robot.h"
#include "robot_env.h"

namespace example_13
{
using namespace chrono;
using namespace chrono::irrlicht;

const uint_t WINDOW_HEIGHT = 800;
const uint_t WINDOW_WIDTH = 1024;
const real_t DT = 0.002;
const real_t SIM_TIME = 5.0;
const std::string WINDOW_TITLE( "Example 13");

// helper functions to build the various components
auto build_generic_material()
{
auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
material->SetFriction(0.8f);
material->SetRestitution(0.1f);
return material;
}

auto build_floor(std::shared_ptr<chrono::ChContactMaterialNSC> material)
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

auto build_wheel(std::shared_ptr<chrono::ChContactMaterialNSC> material, const chrono::ChVector3d& pos)
{
auto wheel = chrono_types::make_shared<chrono::ChBodyEasyCylinder>(chrono::ChAxis::Y,
WHEEL_RADIUS,WHEEL_WIDTH,
1000, true, true, material);

    wheel->SetPos(pos);
    wheel->EnableCollision(true);
    return wheel;
}

auto build_caster_wheel(std::shared_ptr<ChContactMaterialNSC> material, const chrono::ChVector3d& pos)
{

    auto caster = chrono_types::make_shared<ChBodyEasySphere>(
    CASTER_RADIUS,       // radius
    1000,       // density
    true,       // visualization
    true,       // collision
    material);

    caster->SetPos(pos);
    caster->EnableCollision(true);
    return caster;
}

auto build_caster_joint(std::shared_ptr<chrono::ChBodyEasySphere> caster,
std::shared_ptr<chrono::ChBodyEasyBox> chassis)
{
auto caster_joint = chrono_types::make_shared<ChLinkLockRevolute>();

    caster_joint->Initialize(
        caster,
        chassis,
        ChFrame<>(chrono::ChVector3d(-0.25, 0, CASTER_RADIUS), QUNIT));
    return caster_joint;

}

auto build_motor(std::shared_ptr<chrono::ChBodyEasyCylinder> wheel,
std::shared_ptr<chrono::ChFunctionConst> speed_func,
std::shared_ptr<chrono::ChBodyEasyBox> chassis,
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


void
Robot::build(std::shared_ptr<chrono::ChContactMaterialNSC> material)
{
chassis_ = build_chassis(material);
left_wheel_ = build_wheel(material, chrono::ChVector3d(0, 0.2, 0.1));
right_wheel_ = build_wheel(material, chrono::ChVector3d(0,-0.2, 0.1));
left_wheel_motor_ = build_motor(left_wheel_, chrono_types::make_shared<chrono::ChFunctionConst>(-4.5),
chassis_,
chrono::ChVector3d(0,0.2,0.1));
right_wheel_motor_ = build_motor(right_wheel_, chrono_types::make_shared<chrono::ChFunctionConst>(-4.5),
chassis_,
chrono::ChVector3d(0,-0.2,0.1));
caster_wheel_ = build_caster_wheel(material, chrono::ChVector3d(0.2, 0, 0.05));
caster_link_joint_ = build_caster_joint(caster_wheel_, chassis_);
}


void Robot::add_to_sys(chrono::ChSystemNSC& sys) const
{
sys.Add(chassis_);
sys.Add(left_wheel_);
sys.Add(right_wheel_);
sys.Add(left_wheel_motor_);
sys.Add(right_wheel_motor_);
sys.Add(caster_wheel_);
sys.Add(caster_link_joint_);
}

void
Robot::set_speed(real_t speed)
{
chrono::ChVector3d vel(speed, 0.0, 0.0);
chassis_->SetPosDt(vel);

}

void Robot::reset()
{
chrono::ChVector3d pos(0.0, 0.0, WHEEL_RADIUS + CHASSIS_HEIGHT/2.0 + 0.01);
chassis_ -> SetPos(pos);
left_wheel_ -> SetPos(chrono::ChVector3d(0, 0.2, 0.1));
right_wheel_ -> SetPos(chrono::ChVector3d(0,-0.2, 0.1));
caster_wheel_ -> SetPos(chrono::ChVector3d(0.2, 0, 0.05));
}

void RobotEnv::make(const std::string &version,
const std::unordered_map<std::string, std::any> &make_options,
const std::unordered_map<std::string, std::any> &reset_options)
{

    auto material = build_generic_material();
    robot_.build(material);
    build_system_(material);

    robot_.add_to_sys(sys_);

    this -> set_make_options_(make_options);
    this -> set_reset_options_(reset_options);
    this -> set_version_(version);
    this -> make_created_();

}

time_step_type
RobotEnv::reset()
{
sim_counter_ = 1;
current_time_ = 0.0;
robot_.reset();
robot_.set_speed(0.5);

    auto robot_position  = robot_.position();
    return time_step_type(TimeStepTp::FIRST, 0.0, robot_position);
}


time_step_type
RobotEnv::step(const action_type &/*action*/)
{

    if (sim_counter_ % 600 == 0)
    {
#ifdef BITRL_LOG
BOOST_LOG_TRIVIAL(info)<<"Reset simulation: ";
#endif
return reset();
}

    sys_.DoStepDynamics(DT);
    auto robot_position  = robot_.position();

    sim_counter_++;
    current_time_ += DT;
    return time_step_type(TimeStepTp::MID, 1.0, robot_position);
}

void RobotEnv::build_system_(std::shared_ptr<chrono::ChContactMaterialNSC> material)
{
sys_.SetGravitationalAcceleration(chrono::ChVector3d(0,0,-bitrl::consts::maths::G));
sys_.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    auto floor = build_floor(material);
    sys_.Add(floor);
}


void RobotEnv::simulate()
{

    chrono::irrlicht::ChVisualSystemIrrlicht visual;
    visual.AttachSystem(&sys_);
    visual.SetWindowSize(WINDOW_WIDTH, WINDOW_WIDTH);
    visual.SetWindowTitle(WINDOW_TITLE);
    visual.Initialize();

    visual.AddLogo();
    visual.AddSkyBox();
    visual.AddCamera({0, -2, 1}, {0, 0, 0});
    visual.AddTypicalLights();
    visual.BindAll();

    // we need this
    while (visual.Run())
    {
        // Irrlicht must prepare frame to draw
        visual.BeginScene();

        // .. draw items belonging to Irrlicht scene, if any
        visual.Render();

        // .. draw a grid
        tools::drawGrid(&visual, 0.5, 0.5);
        utils::render::irr_draw_world_axes(visual);

        auto time_step = step( action_type());
#ifdef BITRL_LOG
BOOST_LOG_TRIVIAL(info)<<"At time: "<<current_time_<<" position: "<<time_step.observation();
#endif

        // Irrlicht must finish drawing the frame
        visual.EndScene();
    }
}

}


int main() {

    using namespace example_13;
    RobotEnv env;
    env.make("v1", std::unordered_map<std::string, std::any>(),
    std::unordered_map<std::string, std::any>());
    env.reset();

    env.simulate();
    env.close();
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
@endcode

Running the code above produces the following: 

@code
[2026-03-15 17:49:19.445734] [0x00007fbe766b0800] [info]    At time: 0.002 position: 0.000830566  1.24587e-06  0.160031
[2026-03-15 17:49:19.462184] [0x00007fbe766b0800] [info]    At time: 0.004 position: 0.00166125  2.2391e-06  0.160032
[2026-03-15 17:49:19.472812] [0x00007fbe766b0800] [info]    At time: 0.006 position: 0.00249691  2.28551e-06  0.160009
[2026-03-15 17:49:19.490501] [0x00007fbe766b0800] [info]    At time: 0.008 position: 0.00333187  4.48252e-06  0.159995
[2026-03-15 17:49:19.505314] [0x00007fbe766b0800] [info]    At time: 0.01 position: 0.00416369  6.52889e-06  0.159995
[2026-03-15 17:49:19.522646] [0x00007fbe766b0800] [info]    At time: 0.012 position: 0.00499371  5.18348e-06  0.159992
[2026-03-15 17:49:19.538717] [0x00007fbe766b0800] [info]    At time: 0.014 position: 0.00582212  4.4504e-06  0.159994
[2026-03-15 17:49:19.555803] [0x00007fbe766b0800] [info]    At time: 0.016 position: 0.00664943  5.01632e-06  0.159993
[2026-03-15 17:49:19.573243] [0x00007fbe766b0800] [info]    At time: 0.018 position: 0.00747616  5.43181e-06  0.159994
[2026-03-15 17:49:19.589256] [0x00007fbe766b0800] [info]    At time: 0.02 position: 0.00830281  1.09076e-05  0.159984
[2026-03-15 17:49:19.605600] [0x00007fbe766b0800] [info]    At time: 0.022 position: 0.00912996  1.29684e-05  0.159984
[2026-03-15 17:49:19.623059] [0x00007fbe766b0800] [info]    At time: 0.024 position: 0.00995797  9.89451e-06  0.159999
[2026-03-15 17:49:19.639142] [0x00007fbe766b0800] [info]    At time: 0.026 position: 0.0107875  7.19161e-06  0.159997
[2026-03-15 17:49:19.657061] [0x00007fbe766b0800] [info]    At time: 0.028 position: 0.0116187  9.68353e-06  0.159997
[2026-03-15 17:49:19.673839] [0x00007fbe766b0800] [info]    At time: 0.03 position: 0.0124514  1.10808e-05  0.159997
[2026-03-15 17:49:19.690629] [0x00007fbe766b0800] [info]    At time: 0.032 position: 0.0132852  1.08047e-05  0.160001
[2026-03-15 17:49:19.706856] [0x00007fbe766b0800] [info]    At time: 0.034 position: 0.01412  1.23457e-05  0.159996
[2026-03-15 17:49:19.723398] [0x00007fbe766b0800] [info]    At time: 0.036 position: 0.0149556  1.28846e-05  0.159996
[2026-03-15 17:49:19.741549] [0x00007fbe766b0800] [info]    At time: 0.038 position: 0.0157916  1.17644e-05  0.160001
[2026-03-15 17:49:19.758843] [0x00007fbe766b0800] [info]    At time: 0.04 position: 0.0166281  1.43541e-05  0.159996
[2026-03-15 17:49:19.773935] [0x00007fbe766b0800] [info]    At time: 0.042 position: 0.0174651  2.02247e-05  0.15999
[2026-03-15 17:49:19.791439] [0x00007fbe766b0800] [info]    At time: 0.044 position: 0.0183026  2.4407e-05  0.159992
[2026-03-15 17:49:19.806636] [0x00007fbe766b0800] [info]    At time: 0.046 position: 0.0191408  2.38908e-05  0.159993
[2026-03-15 17:49:19.823233] [0x00007fbe766b0800] [info]    At time: 0.048 position: 0.0199796  2.83747e-05  0.159994
[2026-03-15 17:49:19.839922] [0x00007fbe766b0800] [info]    At time: 0.05 position: 0.0208194  3.02409e-05  0.159994
[2026-03-15 17:49:19.857106] [0x00007fbe766b0800] [info]    At time: 0.052 position: 0.0216601  3.2111e-05  0.159996
[2026-03-15 17:49:19.874365] [0x00007fbe766b0800] [info]    At time: 0.054 position: 0.0225017  3.56516e-05  0.159994
[2026-03-15 17:49:19.889853] [0x00007fbe766b0800] [info]    At time: 0.056 position: 0.0233442  3.70565e-05  0.159993
[2026-03-15 17:49:19.907112] [0x00007fbe766b0800] [info]    At time: 0.058 position: 0.0241875  3.91014e-05  0.159997
[2026-03-15 17:49:19.923881] [0x00007fbe766b0800] [info]    At time: 0.06 position: 0.0250316  3.76584e-05  0.159992
[2026-03-15 17:49:19.940460] [0x00007fbe766b0800] [info]    At time: 0.062 position: 0.0258765  3.69224e-05  0.159992
[2026-03-15 17:49:19.957294] [0x00007fbe766b0800] [info]    At time: 0.064 position: 0.0267219  3.70038e-05  0.159996
[2026-03-15 17:49:19.974156] [0x00007fbe766b0800] [info]    At time: 0.066 position: 0.0275681  3.7808e-05  0.159993
...
@endcode


