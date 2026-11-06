#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/sensors/backends/chrono_ultrasound_backend.h"
#include "bitrl/sensors/ultrasonic_sensor.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <gtest/gtest.h>
#include <memory>
#include <string>

namespace
{

using bitrl::real_t;
using bitrl::sensors::backends::CHRONO_UltrasonicBackend;

const real_t MAX_DISTANCE = 5.0;

std::shared_ptr<chrono::ChBody> add_fixed_box(chrono::ChSystemNSC &system,
                                              const chrono::ChVector3d &position, bool collide)
{
    auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    auto box =
        chrono_types::make_shared<chrono::ChBodyEasyBox>(1.0, 1.0, 1.0, 1000.0, false, collide, material);
    box->SetPos(position);
    box->SetFixed(true);
    system.Add(box);
    return box;
}

real_t read_distance(chrono::ChSystemNSC &system, std::shared_ptr<chrono::ChBody> sensor_body)
{
    CHRONO_UltrasonicBackend backend(system, sensor_body);
    backend.set_max_distance(MAX_DISTANCE);

    // the collision system is populated by the first step
    system.DoStepDynamics(0.001);
    return backend.read_values().at(0);
}

} // namespace

TEST(TestChronoUltrasound, ReadsDistanceToObstacle)
{
    chrono::ChSystemNSC system;
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    auto sensor_body = add_fixed_box(system, chrono::ChVector3d(0.0, 0.0, 0.0), false);

    // the face of the obstacle looking at the sensor is at x = 1.5
    add_fixed_box(system, chrono::ChVector3d(2.0, 0.0, 0.0), true);

    EXPECT_NEAR(read_distance(system, sensor_body), 1.5, 1.0e-2);
}

TEST(TestChronoUltrasound, ReturnsMaxDistanceWhenNothingIsInRange)
{
    chrono::ChSystemNSC system;
    system.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    auto sensor_body = add_fixed_box(system, chrono::ChVector3d(0.0, 0.0, 0.0), false);

    // behind the sensor
    add_fixed_box(system, chrono::ChVector3d(-2.0, 0.0, 0.0), true);

    EXPECT_DOUBLE_EQ(read_distance(system, sensor_body), MAX_DISTANCE);
}

TEST(TestChronoUltrasound, LoadsBothDistanceLimitsFromJson)
{
    CHRONO_UltrasonicBackend backend;
    backend.load_from_json(std::string(SENSORS_DATA_DIR) + "/ultrasonic_chrono_backend.json");

    EXPECT_DOUBLE_EQ(backend.max_distance(), 5.0);
    EXPECT_DOUBLE_EQ(backend.min_distance(), 0.5);
}

TEST(TestChronoUltrasound, DisabledSensorDoesNotReadTheBackend)
{
    // no system or body is attached so reading from the backend would fail
    auto backend = std::make_shared<CHRONO_UltrasonicBackend>();

    bitrl::sensors::UltrasonicSensor sensor(backend, "disabled");
    ASSERT_FALSE(sensor.is_enabled());
    EXPECT_TRUE(sensor.read_values().empty());
}
