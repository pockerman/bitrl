#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/rigid_bodies/chrono_robots/chrono_diff_drive_robot.h"
#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/io/json_file_reader.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <chrono/functions/ChFunctionConst.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMotorRotationSpeed.h>
#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapeCylinder.h>


#ifdef BITRL_LOG
#include <boost/log/trivial.hpp>
#endif

#include <array>
#include <optional>
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
    std::array<real_t, 3> visual_position;
    std::string mass_units;
    std::string visual_shape;
    real_t mass;
    bool fixed;

    Chassis(const json &j);
};

Chassis::Chassis(const json &j)
    :
mass(j["mass"].get<real_t>()),
mass_units(j["mass_units"].get<std::string>()),
fixed(j["fixed"].get<bool>()),
visual_shape(j["visual_shape"].get<std::string>()),
position(),
visual_position()
{
    auto pos = j.at("position");
    for (size_t i = 0; i < 3; ++i)
        position[i] = pos.at(i).get<real_t>();

    auto vis_position = j.at("visual_position");
    for (size_t i = 0; i < 3; ++i)
        visual_position[i] = vis_position.at(i).get<real_t>();
}

// helper struct to read a wheel
struct Wheel
{
    std::array<real_t, 3> position;
    std::string mass_units;
    std::string visual_shape;
    real_t mass;
    real_t width;
    real_t radius;

    Wheel(const json &j);
};

Wheel::Wheel(const json &j)
    :
position(),
mass_units(j["mass_units"].get<std::string>()),
visual_shape(j["visual_shape"].get<std::string>()),
mass(j["mass"].get<real_t>()),
width(j["width"].get<real_t>()),
radius(j["radius"].get<real_t>())

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

    // add visual shape for visualization
    auto vis_shape = chrono_types::make_shared<chrono::ChVisualShapeBox>(
    chrono::ChVector3d(chassis_data.visual_position[0], chassis_data.visual_position[1], chassis_data.visual_position[2]));
    chassis -> AddVisualShape(vis_shape);

    return chassis;
}

auto build_wheel(const utils::io::JSONFileReader& json_reader, const std::string &wheel_label)
{

    // read the wheel daat
    auto wheel_data = json_reader.template at<Wheel>(wheel_label);

    auto material = chrono_types::make_shared<chrono::ChContactMaterialSMC>();

    material->SetYoungModulus(2e7);     // stiffness (important)
    material->SetPoissonRatio(0.3);
    material->SetFriction(0.9f);        // traction
    material->SetRestitution(0.0f);     // no bouncing

    // Optional but recommended
    material->SetAdhesion(0.0);
    material->SetKn(2e5);               // normal stiffness override
    material->SetGn(40.0);              // normal damping
    material->SetKt(2e5);               // tangential stiffness
    material->SetGt(20.0);

    // rotation axis for the wheel
    chrono::ChQuaternion<> q;
    q.SetFromAngleAxis(chrono::CH_PI_2, chrono::ChVector3d(1, 0, 0));

    auto wheel = chrono_types::make_shared<chrono::ChBody>();
    wheel->SetMass(wheel_data.mass);
    wheel->SetPos(chrono::ChVector3d(wheel_data.position[0], wheel_data.position[1], wheel_data.position[2]));
    wheel->SetName(wheel_label);
    wheel->SetRot(q);
    wheel->EnableCollision(true);

    auto cyl_shape = chrono_types::make_shared<chrono::ChCollisionShapeCylinder>(
    material,
    wheel_data.radius,
    wheel_data.width * 0.5   // half-length
    );
    wheel->AddCollisionShape(cyl_shape);

    wheel->AddVisualShape(
    chrono_types::make_shared<chrono::ChVisualShapeCylinder>(wheel_data.radius, wheel_data.width));
    return wheel;
}

CHRONO_DiffDriveRobotBase::MotorHandle build_motor(std::shared_ptr<chrono::ChBody> wheel,
                 std::shared_ptr<chrono::ChBody> chassis,
                 const std::string &motor_label)
{


    //chrono::ChVector3d axis = chrono::VECT_Y;

    // Build joint frame in ABSOLUTE coordinates
    chrono::ChQuaterniond q;
    q.SetFromAngleAxis(consts::maths::PI * 0.5, chrono::VECT_X);
    chrono::ChFrame<> frame(wheel->GetPos(), q);

    auto motor =  chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
    motor->Initialize(
    wheel,
    chassis,
    frame);
    motor->SetName(motor_label);

    // set the speed function (rad/s)
    auto speed_func = chrono_types::make_shared<chrono::ChFunctionConst>(0.0);
    motor -> SetSpeedFunction(speed_func);
    return {motor, speed_func};
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

    auto material = chrono_types::make_shared<chrono::ChContactMaterialSMC>();

    material->SetYoungModulus(2e7);     // stiffness (important)
    material->SetPoissonRatio(0.3);
    material->SetFriction(0.9f);        // traction
    material->SetRestitution(0.0f);     // no bouncing

    // Optional but recommended
    material->SetAdhesion(0.0);
    material->SetKn(2e5);               // normal stiffness override
    material->SetGn(40.0);              // normal damping
    material->SetKt(2e5);               // tangential stiffness
    material->SetGt(20.0);
    auto ground = chrono_types::make_shared<chrono::ChBodyEasyBox>(
        5.0, 5.0, 0.1,
        1000,
        true,   // visual
        true,   // collision
        material
    );
    ground->SetFixed(true);


    // set the gravity acceleration
    sys_.SetGravitationalAcceleration(chrono::ChVector3d(0, 0.0, -9.81));

    chassis_ = build_chassis(json_reader);
    sys_.Add(chassis_);
    //sys_.Add(ground);

    name_ = json_reader.template get_value<std::string>("name");

    BOOST_LOG_TRIVIAL(info)<<"Building wheels...: ";

    auto left_wheel = build_wheel(json_reader, "left-wheel");
    auto right_wheel = build_wheel(json_reader, "right-wheel");

    sys_.Add(left_wheel);
    sys_.Add(right_wheel);

    auto left_motor = build_motor(left_wheel, chassis_, "left-motor");
    auto right_motor = build_motor(right_wheel, chassis_, "right-motor");

    sys_.AddLink(left_motor.motor);
    sys_.AddLink(right_motor.motor);
#ifdef BITRL_LOG
    auto zleft = left_motor.motor->GetFrame2Abs().GetRot().GetAxisZ();
    BOOST_LOG_TRIVIAL(info)<<"Left motor rotation: " << zleft;

    auto zright = left_motor.motor->GetFrame2Abs().GetRot().GetAxisZ();
    BOOST_LOG_TRIVIAL(info)<<"Right motor rotation: " << zright;
#endif

    motors_ = std::make_pair<MotorHandle, MotorHandle>(std::move(left_motor), std::move(right_motor));

    auto caster = build_wheel(json_reader, "caster-wheel");
    sys_.Add(caster);
    auto caster_joint = chrono_types::make_shared<chrono::ChLinkLockSpherical>();
    caster_joint->Initialize(
        caster,
        chassis_,
        chrono::ChFrame<>(chrono::ChCoordsys<>(caster->GetPos()))
    );
    sys_.Add(caster_joint);

    // set upd the state
    pose_ = std::make_shared<pose_type>(chassis_);

#ifdef BITRL_LOG
    BOOST_LOG_TRIVIAL(info)<<"Loaded robot: " << name_;
    BOOST_LOG_TRIVIAL(info)<<"Bodies in loaded robot " << sys_.GetBodies().size();
#endif
}

void CHRONO_DiffDriveRobotBase::set_motor_speed(const std::string& motor_name, real_t speed)
{
    if (motor_name == "left-motor")
    {
        motors_.first.speed -> SetConstant(speed);
    }

    if (motor_name == "right-motor")
    {
        motors_.second.speed -> SetConstant(speed);
    }
}

void CHRONO_DiffDriveRobotBase::step(real_t time_step)
{
    sys_.DoStepDynamics(time_step);
}
}
}
#endif

