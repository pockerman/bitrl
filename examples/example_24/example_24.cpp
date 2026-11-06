#include "bitrl/bitrl_config.h"

#if defined(BITRL_CHRONO) && defined(BITRL_IRRLICHT)

#include "bitrl/bitrl_consts.h"
#include "bitrl/rigid_bodies/chrono_robots/chrono_robot_pose.h"
#include "bitrl/utils/bitrl_utils.h"
#include "bitrl/worlds/chrono_worlds/chrono_world_1.h"

#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"
#include "bitrl/utils/render/irrlicht_utils.h"

#include <chrono/physics/ChSystemNSC.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#ifdef BITRL_LOG
#include <boost/log/trivial.hpp>
#endif

#include <string>
#include <filesystem>

namespace example24
{

using namespace bitrl;
using bitrl::worlds::chrono_worlds::ChronoWorld1;

}

int main()
{
    using namespace example24;


#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Starting simulation...: ";
#endif

    // create the system and attach the robot to it
    chrono::ChSystemNSC system;
    system.SetGravitationalAcceleration(chrono::ChVector3d(0,0,-bitrl::consts::maths::G));
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    //auto floor = build_floor(build_generic_material());
    //system.Add(floor);

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Building world...: ";
#endif
    ChronoWorld1 chw1;
    auto path = bitrl::consts::build_world_file_path("chrono_world_1.json");
    chw1.load(path);
    assign_to_chrono_system(system, chw1);

    // create the visualization system
    chrono::irrlicht::ChVisualSystemIrrlicht vis;
    vis.AttachSystem(&system);
    vis.SetWindowSize(1280,720);
    vis.SetWindowTitle("Example 24");
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
