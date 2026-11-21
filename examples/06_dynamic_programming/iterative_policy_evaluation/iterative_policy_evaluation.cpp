#include "bitrl/rl/algorithms/dp/iterative_policy_evaluation.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/policies/uniform_discrete_policy.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"
#include <fstream>
#include <vector>

#include "bitrl/base/bitrl_log.h"
#include "bitrl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "bitrl/network/rest_rl_env_client.h"

#include <iostream>

namespace example_12
{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::envs::gymnasium::FrozenLake;
using bitrl::network::RESTRLEnvClient;
using bitrl::rl::RLSerialAgentTrainer;
using bitrl::rl::RLSerialTrainerConfig;
using bitrl::rl::algos::dp::IterativePolicyEvalConfig;
using bitrl::rl::algos::dp::IterativePolicyEvalutationSolver;
using bitrl::rl::policies::UniformDiscretePolicy;
typedef FrozenLake<4> env_type;

} // namespace example_12

int main()
{

    using namespace example_12;

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

    UniformDiscretePolicy policy(env.n_states(), env.n_actions());
    IterativePolicyEvalConfig config;
    config.tolerance = 1.0e-8;

    IterativePolicyEvalutationSolver<env_type, UniformDiscretePolicy> algorithm(config, policy);

    // the sweeps, as RLSerialAgentTrainer would run them, keeping the value
    // function after every one in sweeps.csv, which examples/tools/value_gif.py
    // draws
    std::ofstream sweeps("sweeps.csv");
    std::vector<real_t> reward;
    algorithm.actions_before_training_begins(env);
    for (uint_t sweep = 0; sweep < 10000; ++sweep)
    {
        const auto sweep_info = algorithm.on_training_episode(env, sweep);
        reward.push_back(sweep_info.episode_reward);
        const auto values = algorithm.value_function();
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
    algorithm.actions_after_training_ends(env);
    BITRL_LOG_INFO << "Done after " << reward.size() << " sweeps";

    // save the value function for plotting
    algorithm.save("iterative_policy_evaluation_frozen_lake.csv");

    return 0;
}
