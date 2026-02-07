#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"


#ifdef BITRL_LOG
#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#endif

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
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

void prepare_visualization(chrono::irrlicht::ChVisualSystemIrrlicht& visual)
{
    visual.SetWindowSize(WINDOW_WIDTH, WINDOW_WIDTH); //WINDOW_HEIGHT);
    visual.SetWindowTitle(WINDOW_TITLE);
    visual.Initialize();

    visual.AddLogo();
    visual.AddSkyBox();
    visual.AddCamera({0, -2, 1}, {0, 0, 0});
    visual.AddTypicalLights();
    visual.BindAll();
}

} // namespace example_13

int main()
{
    using namespace example_13;
    chrono::ChSystemNSC sys;
    sys.SetGravityY();

    // 2- Create the rigid bodies of the slider-crank mechanical system
    //   (a crank, a rod, a truss), maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the truss
    auto my_body_A = chrono_types::make_shared<chrono::ChBody>();
    sys.AddBody(my_body_A);
    my_body_A->SetFixed(true);  // truss does not move!
    my_body_A->SetName("Ground-Truss");

    // ..the crank
    auto my_body_B = chrono_types::make_shared<chrono::ChBody>();
    sys.AddBody(my_body_B);
    my_body_B->SetPos(chrono::ChVector3d(1, 0, 0));  // position of COG of crank
    my_body_B->SetMass(2);
    my_body_B->SetName("Crank");

    // ..the rod
    auto my_body_C = chrono_types::make_shared<chrono::ChBody>();
    sys.AddBody(my_body_C);
    my_body_C->SetPos(chrono::ChVector3d(4, 0, 0));  // position of COG of rod
    my_body_C->SetMass(3);
    my_body_C->SetName("Rod");

    // 3- Create constraints: the mechanical joints between the rigid bodies.

    // .. a revolute joint between crank and rod
    auto my_link_BC = chrono_types::make_shared<chrono::ChLinkLockRevolute>();
    my_link_BC->SetName("RevJointCrankRod");
    my_link_BC->Initialize(my_body_B, my_body_C, chrono::ChFrame<>(chrono::ChVector3d(2, 0, 0)));
    sys.AddLink(my_link_BC);

    // .. a slider joint between rod and truss
    auto my_link_CA = chrono_types::make_shared<chrono::ChLinkLockPointLine>();
    my_link_CA->SetName("TransJointRodGround");
    my_link_CA->Initialize(my_body_C, my_body_A, chrono::ChFrame<>(chrono::ChVector3d(6, 0, 0)));
    sys.AddLink(my_link_CA);

    // .. a motor between crank and truss
    auto my_link_AB = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
    my_link_AB->Initialize(my_body_A, my_body_B, chrono::ChFrame<>(chrono::ChVector3d(0, 0, 0)));
    my_link_AB->SetName("RotationalMotor");
    sys.AddLink(my_link_AB);
    auto my_speed_function = chrono_types::make_shared<chrono::ChFunctionConst>(chrono::CH_PI);  // speed w=3.145 rad/sec
    my_link_AB->SetSpeedFunction(my_speed_function);

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

        // .. draw the rod (from joint BC to joint CA)
        tools::drawSegment(&visual, my_link_BC->GetMarker1()->GetAbsCoordsys().pos,
                           my_link_CA->GetMarker1()->GetAbsCoordsys().pos,
                           chrono::ChColor(0, 1, 0));
        // .. draw the crank (from joint AB to joint BC)
        tools::drawSegment(&visual, my_link_AB->GetFrame2Abs().GetCoordsys().pos,
                           my_link_BC->GetMarker1()->GetAbsCoordsys().pos,
                           chrono::ChColor(1, 0, 0));

        // .. draw a small circle at crank origin
        tools::drawCircle(&visual, 0.1, chrono::ChCoordsys<>(chrono::ChVector3d(0, 0, 0), chrono::QUNIT));

        /* test: delete a link after 10 seconds
        if (sys.GetChTime() >10 && (!removed))
        {
                sys.RemoveLink(my_link_AB);
                removed = true;
        }*/

        // ADVANCE SYSTEM STATE BY ONE STEP
        sys.DoStepDynamics(DT);
        // Enforce soft real-time
        realtime_timer.Spin(DT);

        // Irrlicht must finish drawing the frame
        visual.EndScene();
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