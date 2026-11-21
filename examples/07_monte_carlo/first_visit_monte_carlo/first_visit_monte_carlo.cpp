#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/vector_math.h"
#include "bitrl/rl/algorithms/mc/first_visit_mc.h"
#include "bitrl/rl/learning_rate_scheduler.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"

#include "bitrl/envs/envs_utils.h"
#include "bitrl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "bitrl/network/rest_rl_env_client.h"

#include <fstream>
#include <iostream>
#include <random>

namespace example_22
{

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::envs::gymnasium::FrozenLake;
using bitrl::network::RESTRLEnvClient;
using bitrl::rl::InverseDecayLRScheduler;
using bitrl::rl::RLSerialAgentTrainer;
using bitrl::rl::RLSerialTrainerConfig;
using bitrl::rl::algos::mc::FirstVisitMCSolver;
using bitrl::rl::algos::mc::FirstVisitMCSolverConfig;
using bitrl::rl::policies::EpsilonGreedyPolicy;

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const uint_t SEED = 42;

struct RandomActionSelector
{
    template <typename StateType>
    FrozenLake<4>::action_type operator()(const StateType state) const;
};

// One generator for the whole run. A generator made in every call would
// start from the same seed each time and always pick the same action.
std::mt19937 generator(SEED);

template <typename StateType>
FrozenLake<4>::action_type RandomActionSelector::operator()(const StateType /*state*/) const
{
    // FrozenLake has 4 actions, 0 to 3
    std::uniform_int_distribution<> action_dist(0, 3);
    return action_dist(generator);
}

struct TrajectoryGenerator
{
    std::vector<FrozenLake<4>::time_step_type> operator()(FrozenLake<4> &, uint_t max_steps);
};

std::vector<FrozenLake<4>::time_step_type> TrajectoryGenerator::operator()(FrozenLake<4> &env,
                                                                           uint_t max_steps)
{

    RandomActionSelector action_selector;
    return bitrl::envs::create_trajectory(env, action_selector, max_steps);
}

struct DiscountGenerator
{

    template <typename TrajectoryType>
    std::vector<real_t> operator()(const TrajectoryType &, uint_t max_steps);
};

template <typename TrajectoryType>
std::vector<real_t> DiscountGenerator::operator()(const TrajectoryType &, uint_t max_steps)
{
    return bitrl::maths::logspace(0.0, static_cast<real_t>(max_steps), max_steps, 1.0);
}

typedef FrozenLake<4> env_type;
typedef TrajectoryGenerator trajectory_generator_type;
// alpha / (1 + episode / 100), from the solver's initial 0.5: a constant learning rate
// would keep the estimates moving with every return
typedef InverseDecayLRScheduler learning_rate_scheduler_type;
typedef FirstVisitMCSolverConfig solver_config_type;
typedef DiscountGenerator discount_generator_type;
typedef FirstVisitMCSolver<env_type, trajectory_generator_type, learning_rate_scheduler_type,
                           discount_generator_type>
    solver_type;

} // namespace example_22

int main()
{

    using namespace example_22;

    RESTRLEnvClient server(SERVER_URL, true);

    // create the environment
    FrozenLake<4> env(server);

    std::cout << "Environment URL: " << env.get_url() << std::endl;
    std::unordered_map<std::string, std::any> options;
    std::unordered_map<std::string, std::any> reset_options;

    std::cout << "Creating the environment..." << std::endl;
    env.make("v1", options, reset_options);
    env.reset();
    std::cout << "Done..." << std::endl;

    solver_config_type config;
    config.max_steps = 200;
    learning_rate_scheduler_type lr_scheduler;
    trajectory_generator_type trajectory_generator;
    discount_generator_type discount_generator;
    solver_type solver(config, trajectory_generator, lr_scheduler, discount_generator);

    // the episodes, as RLSerialAgentTrainer would run them, keeping the value
    // function every 200 episodes in sweeps.csv, which
    // examples/tools/value_gif.py draws
    const uint_t N_EPISODES = 10000;
    std::ofstream sweeps("sweeps.csv");
    solver.actions_before_training_begins(env);
    for (uint_t episode = 0; episode < N_EPISODES; ++episode)
    {
        const auto info = solver.on_training_episode(env, episode);
        solver.actions_after_episode_ends(env, episode, info);
        if ((episode + 1) % 200 == 0)
        {
            const auto values = solver.value_function();
            // the episode, from 0, as the sweeps of the other examples
            sweeps << episode;
            for (Eigen::Index s = 0; s < values.size(); ++s)
            {
                sweeps << "," << values[s];
            }
            sweeps << "\n";
        }
    }
    solver.actions_after_training_ends(env);

    // the value of each state under the random policy
    solver.save("first_visit_mc_frozen_lake.csv");

    return 0;
}
