#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/vector_math.h"
#include "bitrl/rl/algorithms/mc/first_visit_mc.h"
#include "bitrl/rl/learning_rate_scheduler.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"

#include "bitrl/envs/envs_utils.h"
#include "bitrl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "bitrl/network/rest_rl_env_client.h"

#include <iostream>
#include <random>

namespace example_22
{

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::envs::gymnasium::FrozenLake;
using bitrl::network::RESTRLEnvClient;
using bitrl::rl::ConstantLRScheduler;
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
typedef ConstantLRScheduler learning_rate_scheduler_type;
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

    RLSerialTrainerConfig trainer_config = {10, 10000, 1.0e-8};
    RLSerialAgentTrainer<env_type, solver_type> trainer(trainer_config, solver);

    auto info = trainer.train(env);
    std::cout << "Trainer info: " << std::endl;
    std::cout << "\t" << info << std::endl;

    // the value of each state under the random policy
    solver.save("first_visit_mc_frozen_lake.csv");

    // save the value function for plotting
    // algorithm.save("iterative_policy_evaluation_frozen_lake.csv");

    return 0;
}
