#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/rigid_bodies/chrono_robots/chrono_diff_drive_robot.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/io/json_file_reader.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <chrono/functions/ChFunctionConst.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMotorRotationSpeed.h>

#ifdef BITRL_LOG
#include <boost/log/trivial.hpp>
#endif

#include <array>
#include <string>
#include <memory>


namespace bitrl
{
namespace rb::bitrl_chrono
{
namespace
{

using json = nlohmann::json;



// helper class to read chassis
struct Chassis
{
    std::array<real_t, 3> position;
    std::string mass_units;
    real_t mass;
    bool fixed;

    Chassis(const json &j);
};

Chassis::Chassis(const json &j)
    :
mass(j["mass"].get<real_t>()),
mass_units(j["mass_units"].get<std::string>()),
fixed(j["fixed"].get<bool>()),
position()
{
    auto pos = j.at("position");
    for (size_t i = 0; i < 3; ++i)
        position[i] = pos.at(i).get<real_t>();
}

// helper struct to read a wheel
struct Wheel
{
    std::array<real_t, 3> position;
    std::string mass_units;
    real_t mass;

    Wheel(const json &j);
};

Wheel::Wheel(const json &j)
    :
mass(j["mass"].get<real_t>()),
mass_units(j["mass_units"].get<std::string>()),
position()
{
    auto pos = j.at("position");
    for (size_t i = 0; i < 3; ++i)
        position[i] = pos.at(i).get<real_t>();
}


auto build_chassis(const utils::io::JSONFileReader& json_reader)
{
    auto chassis_data = json_reader.template at<Chassis>("chassis");
    auto chassis = chrono_types::make_shared<chrono::ChBody>();
    chassis->SetMass(chassis_data.mass);
    chassis->SetInertiaXX(chrono::ChVector3d(0.1, 0.1, 0.1));
    chassis->SetPos(chrono::ChVector3d(chassis_data.position[0], chassis_data.position[1], chassis_data.position[2]));
    chassis->SetFixed(false);
    return chassis;
}

auto build_wheel(const utils::io::JSONFileReader& json_reader, const std::string &wheel_label)
{

    auto wheel_data = json_reader.template at<Wheel>(wheel_label);
    auto wheel = chrono_types::make_shared<chrono::ChBody>();
    wheel->SetMass(wheel_data.mass);
    wheel->SetPos(chrono::ChVector3d(wheel_data.position[0], wheel_data.position[1], wheel_data.position[2]));
    wheel->SetName(wheel_label);
    return wheel;
}

auto build_motor(std::shared_ptr<chrono::ChBody> wheel,
                 std::shared_ptr<chrono::ChBody> chassis,
                 const std::string &motor_label)
{
    chrono::ChQuaterniond q;
    q.SetFromAngleAxis(consts::maths::PI * 0.5, chrono::VECT_X);
    chrono::ChFrame<> frame(wheel->GetPos(), q);
   auto motor =  chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
    motor->Initialize(
    wheel,
    chassis,
    frame);
    motor->SetName(motor_label);

    // set the speed function
    motor -> SetSpeedFunction(chrono_types::make_shared<chrono::ChFunctionConst>(0.0));
    return motor;
}

}

void
CHRONO_DiffDriveRobotBase::load_from_json(const std::string& filename)
{
#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loading robot from file: " << filename;
#endif
    utils::io::JSONFileReader json_reader(filename);
    json_reader.open();

    // set the gravity acceleration
    sys_.SetGravitationalAcceleration(chrono::ChVector3d(0, 0.0, -9.81));

    auto chassis = build_chassis(json_reader);
    sys_.Add(chassis);


    name_ = json_reader.template get_value<std::string>("name");

    auto left_wheel = build_wheel(json_reader, "left-wheel");
    auto right_wheel = build_wheel(json_reader, "right-wheel");

    sys_.Add(left_wheel);
    sys_.Add(right_wheel);

    auto left_motor = build_motor(left_wheel, chassis, "left-motor");
    auto right_motor = build_motor(right_wheel, chassis, "right-motor");

    sys_.Add(left_motor);
    sys_.Add(right_motor);

    auto caster = build_wheel(json_reader, "caster-wheel");
    sys_.Add(caster);
    auto caster_joint = chrono_types::make_shared<chrono::ChLinkLockSpherical>();
    caster_joint->Initialize(
        caster,
        chassis,
        chrono::ChFrame<>(chrono::ChCoordsys<>(caster->GetPos()))
    );
    sys_.Add(caster_joint);

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loaded robot: " << name_;
#endif
}
}
}
#endif

