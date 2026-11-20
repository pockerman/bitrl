
#include "bitrl/envs/grid_world/grid_world_env.h"
#include "bitsim/bitsim_consts.h"
#include "bitsim/bitsim_types.h"

#ifdef BITSIM_DEBUG
#include <cassert>
#endif

#include <any>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>

namespace grid_world
{

using namespace bitsim;

using namespace bitrl::envs::grid_world;

void create_static()
{

    std::cout << "Creating STATIC Gridworld..." << std::endl;

    Gridworld<4> env;

    std::unordered_map<std::string, std::any> options;
    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});
    options["mode"] = std::any(GridWorldInitType::STATIC);

    env.make("v0", options, reset_ops);

    std::cout << "Number of actions: " << env.n_actions() << std::endl;
    std::cout << "Number of states:  " << env.n_states() << std::endl;
    std::cout << "Version:  " << env.version() << std::endl;
    std::cout << "Name:  " << env.env_name() << std::endl;

    auto time_step = env.step(0);
    std::cout << "Time step:" << time_step << std::endl;

    time_step = env.step(1);
    std::cout << "Time step:" << time_step << std::endl;
    env.close();
}

void create_random()
{

    std::cout << "Creating RANDOM Gridworld..." << std::endl;

    bitrl::envs::grid_world::Gridworld<4> env;

    std::unordered_map<std::string, std::any> options;
    options["mode"] = std::any(GridWorldInitType::RANDOM);

    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});

    env.make("v0", options, reset_ops);

    std::cout << "Number of actions: " << env.n_actions() << std::endl;
    std::cout << "Number of states:  " << env.n_states() << std::endl;
    std::cout << "Version:  " << env.version() << std::endl;
    std::cout << "Name:  " << env.env_name() << std::endl;
    env.close();
}

} // namespace grid_world

int main()
{

    using namespace grid_world;
    create_static();
    create_random();
    return 0;
}
