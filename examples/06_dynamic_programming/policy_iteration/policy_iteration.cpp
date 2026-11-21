#include "bitrl/rl/algorithms/dp/policy_iteration.h"
#include "bitrl/base/bitrl_log.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/rl/policies/uniform_discrete_policy.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"

#include <fstream>
#include <iostream>
#include <vector>

namespace rl_example_7
{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string SOLUTION_FILE = "policy_iteration_frozen_lake_v0.csv";

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::envs::gymnasium::FrozenLake;
using bitrl::network::RESTRLEnvClient;
using bitrl::rl::RLSerialAgentTrainer;
using bitrl::rl::RLSerialTrainerConfig;
using bitrl::rl::algos::dp::PolicyIterationConfig;
using bitrl::rl::algos::dp::PolicyIterationSolver;
using bitrl::rl::policies::UniformDiscretePolicy;

typedef FrozenLake<4> env_type;
} // namespace rl_example_7

int main()
{

    BITRL_LOG_INFO << "Starting agent training";
    using namespace rl_example_7;

    RESTRLEnvClient server(SERVER_URL, true);

    // create the environment
    FrozenLake<4> env(server);
    std::unordered_map<std::string, std::any> options;
    std::unordered_map<std::string, std::any> reset_options;
    BITRL_LOG_INFO << "Creating environment...";

    env.make("v1", options, reset_options);
    env.reset();
    BITRL_LOG_INFO << "Done...";

    UniformDiscretePolicy policy(env.n_states(), env.n_actions());

    PolicyIterationConfig config;
    config.gamma = 1.0;
    config.n_policy_eval_steps = 15;
    config.tolerance = 1.0e-8;

    typedef PolicyIterationSolver<env_type, UniformDiscretePolicy> solver_type;

    solver_type solver(config, env.n_actions(), policy);

    // output message frequence, number of episodes, tolerance
    // the sweeps, as RLSerialAgentTrainer would run them, keeping the value
    // function after every one in sweeps.csv, which examples/tools/value_gif.py
    // draws
    std::ofstream sweeps("sweeps.csv");
    std::vector<real_t> reward;
    solver.actions_before_training_begins(env);
    for (uint_t sweep = 0; sweep < 100; ++sweep)
    {
        const auto sweep_info = solver.on_training_episode(env, sweep);
        reward.push_back(sweep_info.episode_reward);
        const auto values = solver.value_function();
        sweeps << sweep;
        for (Eigen::Index s = 0; s < values.size(); ++s)
        {
            sweeps << "," << values[s];
        }
        sweeps << "\n";
        if (sweep_info.stop_training)
        {
            break;
        }
    }
    solver.actions_after_training_ends(env);
    BITRL_LOG_INFO << "Done after " << reward.size() << " sweeps";

    BITRL_LOG_INFO << "Saving solution to " << SOLUTION_FILE;

    // save the value function into a csv file
    solver.save(SOLUTION_FILE);

    BITRL_LOG_INFO << "Finished agent training";
    return 0;
}
