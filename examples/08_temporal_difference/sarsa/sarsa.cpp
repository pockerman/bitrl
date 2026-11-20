#include "bitrl/rl/algorithms/td/sarsa.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/gymnasium/toy_text/cliff_world_env.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"

#include <boost/log/trivial.hpp>
#include <iostream>

namespace rl_example_9
{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string SOLUTION_FILE = "sarsa_cliff_walking_v1.csv";
const std::string REWARD_PER_ITR = "reward_per_itr.csv";
const std::string POLICY = "policy.csv";

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::network::RESTRLEnvClient;
using bitrl::rl::RLSerialAgentTrainer;
using bitrl::rl::RLSerialTrainerConfig;
using bitrl::rl::algos::td::SarsaConfig;
using bitrl::rl::algos::td::SarsaSolver;
using bitrl::rl::policies::EpsilonDecayOption;
using bitrl::rl::policies::EpsilonGreedyPolicy;
typedef bitrl::envs::gymnasium::CliffWorld env_type;

} // namespace rl_example_9

int main()
{

    BOOST_LOG_TRIVIAL(info) << "Starting agent training";
    using namespace rl_example_9;

    try
    {

        RESTRLEnvClient server(SERVER_URL, true);

        // create the environment
        env_type env(server);

        BOOST_LOG_TRIVIAL(info) << "Creating environment...";
        std::unordered_map<std::string, std::any> options;
        std::unordered_map<std::string, std::any> reset_options;
        env.make("v1", options, reset_options);
        env.reset();
        BOOST_LOG_TRIVIAL(info) << "Done...";

        BOOST_LOG_TRIVIAL(info) << "Number of states=" << env.n_states();
        BOOST_LOG_TRIVIAL(info) << "Number of actions=" << env.n_actions();

        // create an e-greedy policy. Use the number
        // of actions as a seed. Use a constant epsilon
        EpsilonGreedyPolicy policy(0.1, env.n_actions(), EpsilonDecayOption::NONE);

        SarsaConfig sarsa_config;
        sarsa_config.gamma = 1.0;
        sarsa_config.eta = 0.5;
        sarsa_config.tolerance = 1.0e-8;
        sarsa_config.max_num_iterations_per_episode = 100;
        sarsa_config.path = SOLUTION_FILE;

        SarsaSolver<env_type, EpsilonGreedyPolicy> algorithm(sarsa_config, policy);

        RLSerialTrainerConfig trainer_config = {10, 2000, 1.0e-8};

        RLSerialAgentTrainer<env_type, SarsaSolver<env_type, EpsilonGreedyPolicy>> trainer(
            trainer_config, algorithm);

        auto info = trainer.train(env);
        BOOST_LOG_TRIVIAL(info) << "Training info..." << info;
        BOOST_LOG_TRIVIAL(info) << "Finished agent training";

        // save the reward the agent achieved per training epoch
        auto reward = trainer.episodes_total_rewards();
        auto iterations = trainer.n_itrs_per_episode();

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
        algorithm.build_policy().save(POLICY);
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    catch (...)
    {

        std::cout << "Unknown exception occured" << std::endl;
    }

    return 0;
}
