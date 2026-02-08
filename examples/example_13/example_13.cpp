#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"


#ifdef BITRL_LOG
#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#endif

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include <chrono/physics/ChBodyEasy.h>
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <filesystem>
#include <iostream>
#include <random>
#include <string>

namespace example_13
{
using namespace bitrl;
using namespace chrono::irrlicht;

// constants we will be using further below
const uint_t WINDOW_HEIGHT = 800;
const uint_t WINDOW_WIDTH = 1024;
const real_t DT = 0.01;
const real_t SIM_TIME = 5.0;
const std::string WINDOW_TITLE( "Example 13");

/*void prepare_visualization(chrono::irrlicht::ChVisualSystemIrrlicht& visual)
{
    visual.SetWindowSize(WINDOW_WIDTH, WINDOW_WIDTH); //WINDOW_HEIGHT);
    visual.SetWindowTitle(WINDOW_TITLE);
    visual.Initialize();

    visual.AddLogo();
    visual.AddSkyBox();
    visual.AddCamera({0, -2, 1}, {0, 0, 0});
    visual.AddTypicalLights();
    visual.BindAll();
}*/


/*class DiffDriveRobot
{
public:

    DiffDriveRobot();

    void set_motor_speed(real_t rad_speed, uint_t id);

    /// Get active drive wheel speed
    chrono::ChVector3d get_wheel_speed(uint_t id);

    /// Get active driver wheel angular velocity
    chrono::ChVector3d get_wheel_angular_velocity(uint_t id);



};*/

} // namespace example_13

int main()
{
   /* using namespace example_13;
    chrono::ChSystemNSC sys;
    sys.SetGravitationalAcceleration(chrono::ChVector3d(0, 0, -9.81));

    sys.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    auto floor_mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    auto mfloor = chrono_types::make_shared<chrono::ChBodyEasyBox>(20, 20, 1, 1000, true, true, floor_mat);
    mfloor->SetPos(chrono::ChVector3d(0, 0, -1));
    mfloor->SetFixed(true);
    mfloor->GetVisualShape(0)->SetTexture(chrono::GetChronoDataFile("textures/concrete.jpg"));
    sys.Add(mfloor);


    chrono::irrlicht::ChVisualSystemIrrlicht visual;
    prepare_visualization(visual);
    visual.AttachSystem(&sys);

    // Simulation loop

    // Timer for enforcing soft real-time
    chrono::ChRealtimeStepTimer realtime_timer;

    // bool removed = false;
    while (visual.Run()) {
        // Irrlicht must prepare frame to draw
        visual.BeginScene();

        // Irrlicht now draws simple lines in 3D world representing a
        // skeleton of the mechanism, in this instant:
        //
        // .. draw items belonging to Irrlicht scene, if any
        visual.Render();
        // .. draw a grid
        tools::drawGrid(&visual, 0.5, 0.5);
        // .. draw GUI items belonging to Irrlicht screen, if any
        visual.GetGUIEnvironment()->drawAll();

        // ADVANCE SYSTEM STATE BY ONE STEP
        sys.DoStepDynamics(DT);
        // Enforce soft real-time
        realtime_timer.Spin(DT);

        // Irrlicht must finish drawing the frame
        visual.EndScene();
    }

    */

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