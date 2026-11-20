#include "bitrl/rl/algorithms/dp/value_iteration.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"

#include "bitrl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitsim/utils/io/csv_file_writer.h"

#include <boost/log/trivial.hpp>
#include <fstream>
#include <tuple>
#include <vector>

namespace example_14
{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string SOLUTION_FILE = "value_iteration_frozen_lake_v1.csv";
const std::string REWARD_PER_ITR = "reward_per_itr.csv";
const std::string POLICY = "policy.csv";

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::rl::RLSerialAgentTrainer;
using bitrl::rl::RLSerialTrainerConfig;
using bitrl::rl::algos::dp::ValueIteration;
using bitrl::rl::algos::dp::ValueIterationConfig;

using bitrl::envs::gymnasium::FrozenLake;
using bitrl::network::RESTRLEnvClient;

// the environment to use
typedef FrozenLake<4> env_type;

// value iteration solver
typedef ValueIteration<env_type> solver_type;

} // namespace example_14

int main()
{

    BOOST_LOG_TRIVIAL(info) << "Starting agent training";

    using namespace example_14;

    RESTRLEnvClient server(SERVER_URL, true);

    // create the environment
    env_type env(server);

    BOOST_LOG_TRIVIAL(info) << "Creating environment...";
    std::unordered_map<std::string, std::any> options;

    options["is_slippery"] = std::any(false);

    std::unordered_map<std::string, std::any> reset_options;
    env.make("v1", options, reset_options);
    env.reset();
    BOOST_LOG_TRIVIAL(info) << "Done...";

    ValueIterationConfig config;
    config.gamma = 0.99;
    config.tolerance = 1.0e-8;

    solver_type algorithm(config);

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
    BOOST_LOG_TRIVIAL(info) << "Done after " << reward.size() << " sweeps";
    BOOST_LOG_TRIVIAL(info) << "Saving solution to " << SOLUTION_FILE;

    // save the value function into a csv file
    algorithm.save(SOLUTION_FILE);

    // save the reward the agent achieved per training epoch

    bitsim::utils::io::CSVWriter csv_writer(REWARD_PER_ITR);
    csv_writer.open();

    csv_writer.write_column_names({"epoch", "reward"});

    auto epoch = static_cast<uint_t>(0);
    for (auto val : reward)
    {

        std::tuple<uint_t, real_t> row = {epoch++, val};
        csv_writer.write_row(row);
    }

    csv_writer.close();

    // build the policy
    algorithm.build_policy(env).save(POLICY);

    BOOST_LOG_TRIVIAL(info) << "Finished agent training";
    return 0;
}
