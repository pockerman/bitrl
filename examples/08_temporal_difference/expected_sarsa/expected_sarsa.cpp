// Expected SARSA on Gymnasium's CliffWalking, with the same settings as
// example 15's SARSA so that the two can be compared
#include "bitrl/rl/algorithms/td/expected_sarsa.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/gymnasium/toy_text/cliff_world_env.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"
#include "bitsim/utils/io/csv_file_writer.h"

#include <boost/log/trivial.hpp>

#include <any>
#include <iostream>
#include <string>
#include <tuple>
#include <unordered_map>

namespace example_26
{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string SOLUTION_FILE = "expected_sarsa_cliff_walking_v1.csv";
const std::string REWARD_PER_ITR = "reward_per_itr.csv";
const std::string POLICY = "policy.csv";

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::network::RESTRLEnvClient;
using bitrl::rl::RLSerialAgentTrainer;
using bitrl::rl::RLSerialTrainerConfig;
using bitrl::rl::algos::td::ExpectedSarsaConfig;
using bitrl::rl::algos::td::ExpectedSarsaSolver;
using bitrl::rl::policies::EpsilonDecayOption;
using bitrl::rl::policies::EpsilonGreedyPolicy;

typedef bitrl::envs::gymnasium::CliffWorld env_type;
typedef ExpectedSarsaSolver<env_type, EpsilonGreedyPolicy> solver_type;

} // namespace example_26

int main()
{
    using namespace example_26;
    BOOST_LOG_TRIVIAL(info) << "Starting agent training";

    try
    {
        RESTRLEnvClient server(SERVER_URL, true);
        env_type env(server);

        BOOST_LOG_TRIVIAL(info) << "Creating environment...";
        std::unordered_map<std::string, std::any> options;
        std::unordered_map<std::string, std::any> reset_options;
        env.make("v1", options, reset_options);
        env.reset();
        BOOST_LOG_TRIVIAL(info) << "Number of states=" << env.n_states();
        BOOST_LOG_TRIVIAL(info) << "Number of actions=" << env.n_actions();

        // an epsilon-greedy policy with a constant epsilon, seeded with the
        // number of actions, as in example 15
        EpsilonGreedyPolicy policy(0.1, env.n_actions(), EpsilonDecayOption::NONE);

        ExpectedSarsaConfig config;
        config.gamma = 1.0;
        config.eta = 0.5;
        config.max_num_iterations_per_episode = 100;
        config.path = SOLUTION_FILE;
        solver_type solver(config, policy);

        RLSerialTrainerConfig trainer_config = {10, 2000, 1.0e-8};
        RLSerialAgentTrainer<env_type, solver_type> trainer(trainer_config, solver);
        auto info = trainer.train(env);
        BOOST_LOG_TRIVIAL(info) << "Training info..." << info;

        // the reward of every training episode
        bitsim::utils::io::CSVWriter csv_writer(REWARD_PER_ITR);
        csv_writer.open();
        csv_writer.write_column_names({"epoch", "reward"});
        uint_t epoch = 0;
        for (auto val : trainer.episodes_total_rewards())
        {
            csv_writer.write_row(std::tuple<uint_t, real_t>{epoch++, val});
        }
        csv_writer.close();

        solver.build_policy().save(POLICY);
        BOOST_LOG_TRIVIAL(info) << "Finished agent training";
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
        return 1;
    }
    return 0;
}
