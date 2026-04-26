#include "bitrl/bitrl_config.h"
#include "bitrl/sensors/messages/ultrasound.h"
#include "bitrl/sensors/ultrasonic_sensor.h"

#ifdef BITRL_CHRONO
#include "bitrl/sensors/backends/chrono_ultrasound_backend.h"
#include "chrono/physics/ChSystemSMC.h"
#include <chrono/physics/ChBodyEasy.h>
#endif

#include <chrono>
#include <iostream>
#include <thread>
#include <iomanip>
#include <memory>

namespace example
{


#ifdef BITRL_CHRONO
void run_chrono()
{
    using namespace bitrl;

    // build a system to use with the backend
    // the system carries the physics
    chrono::ChSystemSMC sys;
    sys.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(chrono::ChVector3d(0, 0, -9.81));

    auto material = chrono_types::make_shared<chrono::ChContactMaterialSMC>();
    material->SetYoungModulus(2e7);
    material->SetFriction(0.5f);
    material->SetRestitution(0.1f);

    // 2. Create ground (target object)
    auto ground = chrono_types::make_shared<chrono::ChBodyEasyBox>(
        2.0, 2.0, 0.1,     // size
        1000,              // density
        true,              // visual
        true,               // collision
        material
    );
    ground->SetPos(chrono::ChVector3d(2.0, 0, 0.2));  // 2m in front
    ground->SetFixed(true);
    sys.Add(ground);

    // 3. Create robot body (sensor mount)
    auto robot = chrono_types::make_shared<chrono::ChBodyEasyBox>(
        0.3, 0.3, 0.2,
        1000,
        true,
        true,
        material
    );
    robot->SetPos(chrono::ChVector3d(0, 0, 0.2));
    robot->SetFixed(true);
    sys.Add(robot);

    sys.DoStepDynamics(0.1);

    auto sensor_backend = std::make_shared<sensors::backends::CHRONO_UltrasonicBackend>(sys, robot);
    sensor_backend -> set_max_distance(5.0);
    sensor_backend -> set_sensor_units("m");

    sensors::UltrasonicSensor sensor(sensor_backend, "MyUltrasonicSensor");
    sensor.init();

    std::cout<<"Sensor name: "<<sensor.sensor_name()<<std::endl;
    std::cout<<"Sensor type: "<<sensor.sensor_type()<<std::endl;
    std::cout<<"Sensor backend: "<<sensor.backend_type_str()<<std::endl;

    for (uint8_t i = 0; i < 10; i++)
    {
        sys.DoStepDynamics(0.1);
        // read values for the sensor
        auto values = sensor.read_values();
        std::cout << "Measured distance: " << values[0] << " in: " <<sensor.sensor_units()<< std::endl;
    }
}
#endif

}

int main()
{

#ifdef BITRL_CHRONO
    example::run_chrono();
#endif

    return 0;
}



