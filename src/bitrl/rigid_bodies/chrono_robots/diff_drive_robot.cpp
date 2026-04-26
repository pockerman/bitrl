#include "bitrl/rigid_bodies/chrono_robots/diff_drive_robot.h"


#ifdef BITRL_CHRONO

#include "bitrl/utils/io/json_file_reader.h"
#include "bitrl/extern/nlohmann/json/json.hpp"
#include "bitrl/sensors/ultrasonic_sensor.h"
#include <chrono/physics/ChContactMaterialNSC.h>

#include <cmath>
#include <vector>
#include <stdexcept>
#include <unordered_map>
#include <any>
#include <iostream>

namespace bitrl
{
namespace rb::bitrl_chrono
{
namespace
{

using json = nlohmann::json;

// helper structs to read data from JSON
struct MaterialData
{
    real_t friction;
    real_t restitution;
    MaterialData(const json& j)
        :
    friction(j.at("friction").get<real_t>()),
    restitution(j.at("restitution").get<real_t>())
    {}
};

struct ChassisData
{
    real_t density{0.0};
    real_t wheel_radius{0.0};
    real_t wheel_width{0.0};
    real_t chassis_x;
    real_t chassis_y;
    real_t chassis_z;
    real_t pos_x;
    real_t pos_y;
    real_t pos_z;

    ChassisData(const json& j)
        :
    chassis_x(j.at("chassis_x").get<real_t>()),
    chassis_y(j.at("chassis_y").get<real_t>()),
    chassis_z(j.at("chassis_z").get<real_t>()),
    pos_x(j.at("pos_x").get<real_t>()),
    pos_y(j.at("pos_y").get<real_t>()),
    pos_z(j.at("pos_z").get<real_t>())
    {}

};

struct WheelData
{
    real_t density;
    real_t wheel_radius;
    real_t wheel_width;
    real_t pos_x;
    real_t pos_y;
    real_t pos_z;

    WheelData(const json& j)
        :
    pos_x(j.at("pos_x").get<real_t>()),
    pos_y(j.at("pos_y").get<real_t>()),
    pos_z(j.at("pos_z").get<real_t>())
    {}
};

struct CasterWheelData
{
    real_t density;
    real_t wheel_radius;
    real_t pos_x;
    real_t pos_y;
    real_t pos_z;
    CasterWheelData(const json& j)
       :
   pos_x(j.at("pos_x").get<real_t>()),
   pos_y(j.at("pos_y").get<real_t>()),
   pos_z(j.at("pos_z").get<real_t>())
    {}
};

// helper functions to build the various entities of the robot
auto build_material(const MaterialData& md)
{
    auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    material->SetFriction(md.friction);
    material->SetRestitution(md.restitution);
    return material;
}

auto build_chassis(std::shared_ptr<chrono::ChContactMaterialNSC> material, const ChassisData& cd)
{
    auto chassis = chrono_types::make_shared<chrono::ChBodyEasyBox>(cd.chassis_x, cd.chassis_y, cd.chassis_z,
                                                                    cd.density, true, true, material);

    chrono::ChVector3d pos(cd.pos_x, cd.pos_y, cd.wheel_radius + cd.chassis_z/2.0 + 0.01);
    chassis->SetPos(pos);
    chassis->EnableCollision(true);
    return chassis;
}

auto build_wheel(std::shared_ptr<chrono::ChContactMaterialNSC> material, const WheelData& wd)
{
    auto wheel = chrono_types::make_shared<chrono::ChBodyEasyCylinder>(chrono::ChAxis::Y,
                                                                       wd.wheel_radius,wd.wheel_width,
                                                                       wd.density, true, true, material);

    const chrono::ChVector3d pos(wd.pos_x, wd.pos_y, wd.pos_z);
    wheel->SetPos(pos);
    wheel->EnableCollision(true);
    return wheel;
}

auto build_caster_wheel(std::shared_ptr<chrono::ChContactMaterialNSC> material,
                        const CasterWheelData& cwd,
                        bool visualization=true,
                        bool collision=true
                        )
{

    auto caster = chrono_types::make_shared<chrono::ChBodyEasySphere>(
    cwd.wheel_radius,       // radius
    cwd.density,       // density
    visualization,       // visualization
    collision,       // collision
    material);

    const chrono::ChVector3d pos(cwd.pos_x, cwd.pos_y, cwd.pos_z);
    caster->SetPos(pos);
    caster->EnableCollision(true);
    return caster;
}

auto build_caster_joint(std::shared_ptr<chrono::ChBodyEasySphere> caster,
                        std::shared_ptr<chrono::ChBodyEasyBox> chassis,
                        chrono::ChVector3d& pos)
{
    auto caster_joint = chrono_types::make_shared<chrono::ChLinkLockRevolute>();
    caster_joint->Initialize(
        caster,
        chassis,
        chrono::ChFrame<>(pos, chrono::QUNIT));
    return caster_joint;

}

auto build_motor(std::shared_ptr<chrono::ChBodyEasyCylinder> wheel,
                 std::shared_ptr<chrono::ChFunctionConst> speed_func,
                 std::shared_ptr<chrono::ChBodyEasyBox> chassis,
                 const chrono::ChVector3d& pos)
{
    auto motor = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
    chrono::ChQuaternion<> rot = chrono::QuatFromAngleX(chrono::CH_PI_2);
    motor->Initialize(
        wheel,
        chassis,
        chrono::ChFrame<>(pos, rot)
    );

    motor->SetSpeedFunction(speed_func);
    return motor;
}

}

template<>
void CHRONO_DiffDriveRobot::RobotData::read(const json& j)
{
    max_speed = j.at("max_speed").get<real_t>();
    wheel_radius = j.at("wheel_radius").get<real_t>();
    wheel_width = j.at("wheel_width").get<real_t>();
    axle_length = j.at("axle_length").get<real_t>();
    density = j.at("density").get<real_t>();
    caster_wheel_radius = j.at("caster_wheel_radius").get<real_t>();
}

namespace
{

using json = nlohmann::json;

struct RangeSensorData
{
    uint_t idx;
    bool enable;
    real_t update_rate;
    real_t max_distance;
    real_t min_distance;
    std::string backend;
    std::string name;
    std::vector<float_t> color;
    std::vector<float_t> frame_pos;
    std::vector<float_t> dimensions;

    RangeSensorData(const json& data)
        :
    idx(data.at("idx").get<uint_t>()),
    enable(data.at("enable").get<bool>()),
    update_rate(data.at("update_rate").get<real_t>()),
    max_distance(data.at("max_distance").get<real_t>()),
    min_distance(data.at("min_distance").get<real_t>()),
    backend(data.at("backend").get<std::string>()),
    name(data.at("name").get<std::string>()),
    color(data.at("color").get<std::vector<float_t>>()),
    frame_pos(data.at("frame_pos").get<std::vector<float_t>>()),
    dimensions(data.at("dimensions").get<std::vector<float_t>>())
    {}

};
}

CHRONO_DiffDriveRobot::RobotSensorManager::RobotSensorManager()
    :
sensor_manager_(0)
{}

template<>
void CHRONO_DiffDriveRobot::RobotSensorManager::read(const json& j, std::shared_ptr<chrono::ChBody> body)
{

    auto sensors = j.at("sensors");
    auto fsdata = RangeSensorData(sensors.at("front-sonar"));

    auto fs = sensors::UltrasonicSensor::create(fsdata.backend, fsdata.name);
    sensor_manager_.add(fs);

    // get the backend and create the visual shape
    auto backend_ptr = fs -> backend_ptr();

    if (!backend_ptr)
    {
        throw std::runtime_error("Backend ptr pointer is not set");
    }

    backend_ptr -> set_chrono_body(body);
    chrono::ChFrame<> sensor_frame(
        chrono::ChVector3d(fsdata.frame_pos[0], fsdata.frame_pos[1], fsdata.frame_pos[1])
    );

    chrono::ChColor color(fsdata.color[0], fsdata.color[1], fsdata.color[2]);
    std::unordered_map<std::string, std::any> vis_shape_properties;
    vis_shape_properties["chrono::ChColor"] = color;
    vis_shape_properties["chrono::ChFrame<>"] = sensor_frame;
    vis_shape_properties["x_length"] = fsdata.dimensions[0];
    vis_shape_properties["y_length"] = fsdata.dimensions[1];
    vis_shape_properties["z_length"] = fsdata.dimensions[2];
    backend_ptr -> add_visual_shape(vis_shape_properties);
}

void CHRONO_DiffDriveRobot::add_to_sys(chrono::ChSystemNSC& sys) const
{
    if (!is_loaded_)
    {
        throw std::logic_error("Robot has not been loaded have you called load?");
    }

    sys.Add(chassis_);
    sys.Add(left_wheel_);
    sys.Add(right_wheel_);
    sys.Add(left_wheel_motor_);
    sys.Add(right_wheel_motor_);
    sys.Add(caster_wheel_);
    sys.Add(caster_link_joint_);
}

void CHRONO_DiffDriveRobot::set_speed(real_t speed)
{
    if (!is_loaded_)
    {
        throw std::logic_error("Robot has not been loaded have you called load?");
    }

    // find out the current velocity vector
    auto current_vel_vec = chassis_ -> GetPosDt();
    chrono::ChVector3d vel(speed, current_vel_vec[1], current_vel_vec[2]);
    chassis_ -> SetPosDt(vel);
    auto speed_ = vel.Length();

    //set the motors speed
    auto omega = speed_ / data_.wheel_radius;
    set_motor_speed(omega);

}

void CHRONO_DiffDriveRobot::load(const std::filesystem::path& filename)
{

    if (is_loaded_)
    {
        return;
    }

    utils::io::JSONFileReader reader(filename);
    reader.open();

    data_.read(reader.get("robot-data"));

    auto md = reader.at<MaterialData>("material");
    auto material = build_material(md);

    auto cd = reader.at<ChassisData>("chassis");
    cd.wheel_radius = data_.wheel_radius;
    cd.wheel_width = data_.wheel_width;
    cd.density = data_.density;
    chassis_ = build_chassis(material, cd);

    auto lwd = reader.at<WheelData>("left-wheel");
    lwd.wheel_radius = data_.wheel_radius;
    lwd.wheel_width = data_.wheel_width;
    lwd.density = data_.density;
    left_wheel_ = build_wheel(material, lwd);

    auto rwd = reader.at<WheelData>("right-wheel");
    rwd.wheel_radius = data_.wheel_radius;
    rwd.wheel_width = data_.wheel_width;
    rwd.density = data_.density;
    right_wheel_ = build_wheel(material, rwd);

    auto cwd = reader.at<CasterWheelData>("caster-wheel");
    cwd.wheel_radius = data_.caster_wheel_radius;
    cwd.density = data_.density;
    caster_wheel_  = build_caster_wheel(material, cwd);

    chrono::ChVector3d pos(-0.25, 0, cwd.wheel_radius);
    caster_link_joint_ = build_caster_joint(caster_wheel_, chassis_, pos);

    left_wheel_motor_speed_func_ = chrono_types::make_shared<chrono::ChFunctionConst>(0.0);
    left_wheel_motor_ = build_motor(left_wheel_,
                                    left_wheel_motor_speed_func_,
                                    chassis_, chrono::ChVector3d(0,0.2,0.1));

    right_wheel_motor_speed_func_ = chrono_types::make_shared<chrono::ChFunctionConst>(0.0);
    right_wheel_motor_ = build_motor(right_wheel_, right_wheel_motor_speed_func_,
                                    chassis_, chrono::ChVector3d(0.0,-0.2,0.1));

    // track the chassis for pose reports
    pose_.set_body(chassis_);

    // read the sensors
    sensor_manager_.read(reader.get("sensors"), chassis_);

    // set the loaded flag to true to signal
    // successful loading
    is_loaded_ = true;
}

void CHRONO_DiffDriveRobot::set_motor_speed(uint_t m, real_t speed)
{
    if (!is_loaded_)
    {
        throw std::logic_error("Robot has not been loaded have you called load?");
    }

    if (m !=0 || m != 1)
    {
        throw std::logic_error("Invalid motor index. Index must be either 0 or 1");
    }

    if (m == 0)
    {
        left_wheel_motor_speed_func_ -> SetConstant(speed);
    }

    if (m == 1)
    {
        right_wheel_motor_speed_func_ -> SetConstant(speed);
    }

}

void CHRONO_DiffDriveRobot::set_motor_speed(real_t speed)
{
    set_motor_speed(0, speed);
    set_motor_speed(1, speed);

    // now calculate the linear velocities of the motors
    auto vl = speed * data_.wheel_radius;
    auto vr = speed * data_.wheel_radius;

    auto v = 0.5 * (vl + vr);
    auto omega = (vr - vl) * data_.axle_length;

    auto vx = v * std::cos(omega);
    auto vy = v * std::sin(omega);

    chrono::ChVector3d vel(vx, vy, 0.0);
    chassis_ -> SetPosDt(vel);
}

}
}


#endif
