#include "bitrl/worlds/chrono_worlds/chrono_world_1.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/io/json_file_reader.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <array>
#include <chrono/assets/ChVisualShapeBox.h>
#include <future>

namespace bitrl
{
namespace worlds::chrono_worlds
{

namespace
{
using json = nlohmann::json;
}


void assign_to_chrono_system(chrono::ChSystemNSC& system, const ChronoWorld1& world)
{

    system.Add(world.box1_);
    system.Add(world.box2_);
    system.Add(world.box3_);
    system.Add(world.floor_);
    world.robot_.add_to_sys(system);
}

void ChronoWorld1::load_boxes_(const json& j)
{
    struct ChronoBox
    {
        real_t x;
        real_t y;
        real_t z;
        bool enable_visualization;
        real_t pos_x;
        real_t pos_y;
        real_t pos_z;
        real_t density;
        real_t mass;

        ChronoBox(const json& j)
        :
        x(j.at("x").get<real_t>()),
        y(j.at("y").get<real_t>()),
        z(j.at("z").get<real_t>()),
        enable_visualization(j.at("enable_visualization").get<bool>()),
        pos_x(j.at("pos_x").get<real_t>()),
        pos_y(j.at("pos_y").get<real_t>()),
        pos_z(j.at("pos_z").get<real_t>()),
        density(j.at("density").get<real_t>()),
        mass(j.at("mass").get<real_t>())
        {}
    };
    {
        auto box_data = ChronoBox(j.at("box1"));
        box1_ = chrono_types::make_shared<chrono::ChBodyEasyBox>(box_data.x,
            box_data.y, box_data.z, box_data.density, box_data.enable_visualization);
        box1_ -> SetMass(box_data.mass);
        box1_ -> SetPos(chrono::ChVector3d(box_data.pos_x, box_data.pos_y, box_data.pos_z));
        box1_ -> SetFixed(true);

        // create visual shape
        auto visual_box = chrono_types::make_shared<chrono::ChVisualShapeBox>(box_data.x, box_data.y, box_data.z);
        visual_box->SetColor(chrono::ChColor(1.0f, 0.0f, 0.0f)); // red

        box1_ ->AddVisualShape(visual_box);

        //box1_ = box;
    }

    {
        auto box_data = ChronoBox(j.at("box2"));
        box2_ = chrono_types::make_shared<chrono::ChBodyEasyBox>(box_data.x,
            box_data.y, box_data.z, box_data.density, box_data.enable_visualization);
        box2_ -> SetMass(box_data.mass);
        box2_ -> SetPos(chrono::ChVector3d(box_data.pos_x, box_data.pos_y, box_data.pos_z));
        box2_ -> SetFixed(true);
    }

    {
        auto box_data = ChronoBox(j.at("box3"));
        box3_ = chrono_types::make_shared<chrono::ChBodyEasyBox>(box_data.x,
            box_data.y, box_data.z, box_data.density, box_data.enable_visualization);
        box3_ -> SetMass(box_data.mass);
        box3_ -> SetPos(chrono::ChVector3d(box_data.pos_x, box_data.pos_y, box_data.pos_z));
        box3_ -> SetFixed(true);
    }
}

std::shared_ptr<chrono::ChContactMaterialNSC>
ChronoWorld1::load_floor_material_(const json& j)
{
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

    auto md = MaterialData(j);
    auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    material->SetFriction(md.friction);
    material->SetRestitution(md.restitution);
    return material;
}

void ChronoWorld1::load_floor_(const json& j)
{
    struct Floor
    {
        real_t x;
        real_t y;
        real_t z;
        bool enable_visualization;
        real_t pos_x;
        real_t pos_y;
        real_t pos_z;
        real_t density;
        real_t mass;

        Floor(const json& j)
        :
        x(j.at("x").get<real_t>()),
        y(j.at("y").get<real_t>()),
        z(j.at("z").get<real_t>()),
        enable_visualization(j.at("enable_visualization").get<bool>()),
        pos_x(j.at("pos_x").get<real_t>()),
        pos_y(j.at("pos_y").get<real_t>()),
        pos_z(j.at("pos_z").get<real_t>()),
        density(j.at("density").get<real_t>()),
        mass(j.at("mass").get<real_t>())
        {}
    };

    auto material = load_floor_material_(j);
    auto floor_data = Floor(j);

    floor_ = chrono_types::make_shared<chrono::ChBodyEasyBox>(
floor_data.x, floor_data.y, floor_data.z,     // size
floor_data.density,          // density
floor_data.enable_visualization, true, material);   // visual + collision

    floor_->SetPos(chrono::ChVector3d(floor_data.pos_x,floor_data.pos_y,floor_data.pos_z));
    floor_->SetFixed(true);
    floor_ -> SetMass(floor_data.mass);
    floor_->EnableCollision(true);
}

void ChronoWorld1::load(const std::filesystem::path& filename)
{

    if (is_loaded_)
    {
        return;
    }

    utils::io::JSONFileReader reader(filename);
    reader.open();

    // We can load the robot as an async task
    auto robot_future = std::async(std::launch::async, [this, &reader]() {
        auto robo_json = reader.get("robot");
        robot_.load(robo_json);
    });

    auto boxes_future = std::async(std::launch::async, [this, &reader]() {
        this -> load_boxes_(reader.get("boxes"));
    });

    auto floor_future = std::async(std::launch::async, [this, &reader]() {
        this -> load_floor_(reader.get("floor"));
    });

    // wait for both futures
    robot_future.get();
    boxes_future.get();
    floor_future.get();

    // move the robot to
    // bottom left corner
    //robot_.set_world_position(std::array{-2.0, -2.0, 0.0});

    is_loaded_ = true;
}

}
}


#endif
