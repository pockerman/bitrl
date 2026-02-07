/*
 * This example shows how to create
 * a waypoint trajectory consisting of straight
 * lines
 *
 *
 *
 */

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_types.h"

#ifdef BITRL_LOG
#define BOOST_LOG_DYN_LINK
#include <boost/log/trivial.hpp>
#endif

#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>


#include "bitrl/utils/geometry/geom_point.h"
#include "bitrl/utils/trajectory/line_segment_link.h"
#include "bitrl/utils/trajectory/waypoint.h"
#include "bitrl/utils/trajectory/waypoint_trajectory.h"
#include "bitrl/utils/unit_converter.h"

#include <any>
#include <array>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace example_11
{

using namespace bitrl;
using namespace chrono::irrlicht;

// constants we will be using further below
const uint_t WINDOW_HEIGHT = 800;
const uint_t WINDOW_WIDTH = 1024;
const real_t DT = 0.0001;
const real_t SIM_TIME = 5.0;
const std::string WINDOW_TITLE( "Example 11");

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

std::shared_ptr<chrono::ChBody> create_box(real_t xlength, real_t ylength, real_t zlength,
                                                 real_t density, bool create_visualization)
{
    auto box = chrono_types::make_shared<chrono::ChBodyEasyBox>(xlength, ylength, zlength,
                                                                                        density, create_visualization);
    box -> SetMass(1.0);
    box -> SetPos(chrono::ChVector3d(0.0, 0.0, 0.22));
    return box;

}

} // namespace example_11

/*
int main()
{

    using namespace example_11;
    chrono::ChSystemSMC sys;

    // build the body and add it to the system
    // we want to simulate
    auto box = create_box(1.0, 1.0, 1.0, 1.0, true);
    sys.Add(box);

    // create the object that handles the visualization
    chrono::irrlicht::ChVisualSystemIrrlicht visual;
    prepare_visualization(visual);
    visual.AttachSystem(&sys);

    while (visual.Run())
    {

        // Irrlicht must prepare frame to draw
        visual.BeginScene();

        // .. draw items belonging to Irrlicht scene, if any
        visual.Render();

        // .. draw a grid
        tools::drawGrid(&visual, 0.5, 0.5);

        // Irrlicht must finish drawing the frame
        visual.EndScene();
    }

}*/

int main()
{

    using namespace example_11;
    chrono::ChSystemSMC sys;

    // build the body and add it to the system
    // we want to simulate
    auto box = create_box(1.0, 1.0, 1.0, 1.0, true);
    sys.Add(box);

    auto position = box -> GetPos();
    BOOST_LOG_TRIVIAL(info)<<"Position of CoM: "<<position;
    BOOST_LOG_TRIVIAL(info)<<"Linear velocity of CoM: "<<box -> GetLinVel();

    // create a frame this is assumed to be the inertial frame we call this
    // the world or reference frame
    chrono::ChFrame ref_frame(chrono::ChVector3d(0.0, 0.0, 0.0));
    BOOST_LOG_TRIVIAL(info)<<"Position of ref frame: "<<ref_frame.GetPos();

    // child frame DEFINED RELATIVE TO ref_frame
    chrono::ChFrame child_frame_in_ref(chrono::ChVector3d(0.0, 0.0, 0.22));
    BOOST_LOG_TRIVIAL(info)<<"Position of translated frame: "<<child_frame_in_ref.GetPos();

    // compose: child frame expressed in WORLD
    chrono::ChFrame translated_in_world = ref_frame * child_frame_in_ref;
    BOOST_LOG_TRIVIAL(info)<<"Position of translated in world frame: "<<translated_in_world.GetPos();


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
