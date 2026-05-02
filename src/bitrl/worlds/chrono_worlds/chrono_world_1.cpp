#include "bitrl/worlds/chrono_worlds/chrono_world_1.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/utils/io/json_file_reader.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <future>

namespace bitrl
{
namespace worlds::chrono_worlds
{

void assign_to_chrono_system(chrono::ChSystemNSC& system, const ChronoWorld1& world)
{

    system.Add(world.box1_);
    system.Add(world.box2_);
    system.Add(world.box3_);
    system.Add(world.box4_);
    system.Add(world.box5_);
    system.Add(world.box6_);
    system.Add(world.floor_);

    world.robot_.add_to_sys(system);
}


void ChronoWorld1::load(const std::filesystem::path& filename)
{

    if (is_loaded_)
    {
        return;
    }

    utils::io::JSONFileReader reader(filename);
    reader.open();

    // read the file to load the robot from
    auto robo_file = reader.at<std::string>("robot_file");
    auto path = consts::build_robot_file_path(robo_file);

    // We can load the robot as an async task
    auto future = std::async(std::launch::async, [this, path]() {
        robot_.load(path);
    });

    // optionally store the future if you need to wait later
    auto load_future_ = std::move(future);

}

}
}


#endif
