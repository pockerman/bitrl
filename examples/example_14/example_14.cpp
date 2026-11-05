#include "bitrl/bitrl_config.h"

#if defined(BITRL_CHRONO) && defined(BITRL_IRRLICHT)

#include "bitrl/bitrl_consts.h"
#include "bitrl/rigid_bodies/chrono_robots/chrono_robot_pose.h"
#include "bitrl/utils/bitrl_utils.h"

#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitrl/utils/render/irrlicht_utils.h"

#include <chrono/physics/ChSystemNSC.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#ifdef BITRL_LOG
#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#endif

#include <string>
#include <filesystem>

namespace example14
{

using namespace bitrl;
using bitrl::rb::bitrl_chrono::CHRONO_DiffDriveRobot;

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

}

int main()
{
    using namespace example14;


#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Starting simulation...: ";
#endif

    // create the system and attach the robot to it
    chrono::ChSystemNSC system;
    system.SetGravitationalAcceleration(chrono::ChVector3d(0,0,-bitrl::consts::maths::G));
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    auto floor = build_floor(build_generic_material());
    system.Add(floor);

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Building robot...: ";
#endif
    CHRONO_DiffDriveRobot robot;
    auto path = std::filesystem::path(bitrl::consts::ROBOTS_DIR + "/bitrl_diff_drive_robot.json");
    robot.load(path);

    robot.add_to_sys(system);

    auto& sensor_manager = robot.get_sensor_manager();
#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Numner of sensors: "<<sensor_manager.n_sensors();
#endif

    // create the visualization system
    chrono::irrlicht::ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&system);
    vis.SetWindowSize(1280,720);
    vis.SetWindowTitle("Example 14 Differential Drive Robot");
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
        utils::render::irr_draw_world_axes(vis, 2.0);

        //auto position = chassis -> GetPos();
        //std::cout << position <<std::endl;

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
