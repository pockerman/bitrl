// Checks that an installed bitrl can be compiled against, linked and run: an
// environment from the library, libtorch through bitrl's link interface, and
// bitsim's types
#include "bitrl/base/bitrl_version.h"
#include "bitrl/envs/grid_world/grid_world_env.h"
#include "bitsim/bitsim_types.h"

#include <torch/torch.h>

#include <any>
#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_map>

int main()
{
    using bitrl::envs::grid_world::Gridworld;
    using bitrl::envs::grid_world::GridWorldInitType;

    Gridworld<4> env;
    std::unordered_map<std::string, std::any> options;
    options["mode"] = std::any(GridWorldInitType::STATIC);
    env.make("v0", options, {});
    auto time_step = env.reset();

    const auto values = torch::ones({static_cast<std::int64_t>(env.n_states())});
    const auto sum = values.sum().item<bitsim::real_t>();

    std::cout << "bitrl " << BITRLLIB_VERSION << ": " << env.n_states() << " states, "
              << env.n_actions() << " actions, reward " << time_step.reward() << std::endl;
    if (static_cast<bitsim::uint_t>(sum) != env.n_states())
    {
        std::cerr << "error: libtorch gave " << sum << std::endl;
        return 1;
    }
    return 0;
}
