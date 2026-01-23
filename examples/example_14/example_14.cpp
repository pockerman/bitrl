#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO


#include "bitrl/bitrl_consts.h"
#include "bitrl/sensors/ultrasonic_sensor.h"
#include "bitrl/rigid_bodies/chrono_robots/chrono_diff_drive_robot.h"
#include "bitrl/sensors/backends/chrono_ultrasound_backend.h"
#include "bitrl/sensors/sensor_manager.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"


namespace example14
{
using namespace bitrl;
using namespace chrono::irrlicht;
using bitrl::rb::bitrl_chrono::CHRONO_DiffDriveRobotBase;
}

int main()
{
    using namespace example14;

    CHRONO_DiffDriveRobotBase robot;
    robot.load_from_json(bitrl::consts::ROBOTS_DIR + "/bitrl_diff_drive_robot.json");

    // create the sensors for the rebot
    auto sensor_backend = std::make_shared<sensors::backends::CHRONO_UltrasonicBackend>(robot.CHRONO_sys(),
                                                                                         robot.CHRONO_chassis());
    sensor_backend -> load_from_json(bitrl::consts::SENSORS_DIR + "/ultrasonic_chrono_backend.json");

    sensors::SensorManager sensor_manager(1);
    sensor_manager.add(std::make_shared<sensors::UltrasonicSensor>(sensor_backend, "UltrasoundSensor-1"));
    robot.set_sensors(sensor_manager);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&robot.CHRONO_sys());
    vis->SetWindowSize(1024, 768);
    vis->SetWindowTitle("Odysseus Robot");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera({0, -2, 1}, {0, 0, 0});

    vis->AddTypicalLights();
    vis->BindAll();

    while (vis->Run())
    {
        // Irrlicht must prepare frame to draw
        vis->BeginScene();

        // .. draw items belonging to Irrlicht scene, if any
        vis->Render();

        // .. draw a grid
        tools::drawGrid(vis.get(), 0.5, 0.5);
        // .. draw GUI items belonging to Irrlicht screen, if any
        vis->GetGUIEnvironment()->drawAll();

        robot.step(0.001);

        // Irrlicht must finish drawing the frame
        vis->EndScene();
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
