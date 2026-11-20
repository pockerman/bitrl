// Double Q-learning on Gymnasium's CliffWalking, with the settings of the
// other temporal difference examples so that they can be compared
//
// Needs the bitrl-rest-api server at SERVER_URL.

#include "bitrl/rl/algorithms/td/double_q_learning.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/gymnasium/toy_text/cliff_world_env.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/rl/policies/epsilon_double_qtable_greedy_policy.h"
#include "bitrl/rl/policies/max_tabular_policy.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"
#include "bitsim/utils/io/csv_file_writer.h"

#include <any>
#include <iostream>
#include <string>
#include <tuple>
#include <unordered_map>

namespace double_q_learning
{

using bitrl::DynMat;
using bitrl::real_t;
using bitrl::uint_t;
using bitrl::rl::EpsilonDecayOptionType;
using bitrl::rl::algos::td::DoubleQLearning;
using bitrl::rl::algos::td::DoubleQLearningConfig;
using bitrl::rl::policies::EpsilonDoubleQTableGreedyPolicy;

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string TABLES_FILE = "double_q_learning_cliff_walking_v1.csv";
const std::string REWARDS_FILE = "reward_per_itr.csv";
const std::string POLICY_FILE = "policy.csv";

typedef bitrl::envs::gymnasium::CliffWorld env_type;
typedef EpsilonDoubleQTableGreedyPolicy<DynMat<real_t>> policy_type;
typedef DoubleQLearning<env_type, policy_type> solver_type;

} // namespace double_q_learning

int main()
{
    using namespace double_q_learning;

    try
    {
        bitrl::network::RESTRLEnvClient server(SERVER_URL, false);
        env_type env(server);
        std::unordered_map<std::string, std::any> options;
        std::unordered_map<std::string, std::any> reset_options;
        env.make("v1", options, reset_options);

        // epsilon-greedy on the sum of the two tables, with a constant epsilon
        policy_type policy(0.1, env.n_actions(), EpsilonDecayOptionType::NONE, 0.01, 1.0, 0.2,
                           env.n_actions());

        DoubleQLearningConfig config;
        config.gamma = 1.0;
        config.eta = 0.5;
        config.max_num_iterations_per_episode = 100;
        config.path = TABLES_FILE;
        solver_type solver(config, policy);

        bitrl::rl::RLSerialTrainerConfig trainer_config{100, 2000, 1.0e-8};
        bitrl::rl::RLSerialAgentTrainer<env_type, solver_type> trainer(trainer_config, solver);
        trainer.train(env);

        // the reward of every training episode
        bitsim::utils::io::CSVWriter rewards(REWARDS_FILE);
        rewards.open();
        rewards.write_column_names({"epoch", "reward"});
        uint_t epoch = 0;
        for (auto val : trainer.episodes_total_rewards())
        {
            rewards.write_row(std::tuple<uint_t, real_t>{epoch++, val});
        }
        rewards.close();

        // the greedy policy of the two tables together, as the policy acts on
        const DynMat<real_t> sum = solver.q_table_1() + solver.q_table_2();
        bitrl::rl::policies::MaxTabularPolicy greedy;
        bitrl::rl::policies::MaxTabularPolicyBuilder().build_from_state_action_function(sum,
                                                                                        greedy);
        greedy.save(POLICY_FILE);
        env.close();
        std::cout << "Saved the tables to " << TABLES_FILE << " and the greedy policy to "
                  << POLICY_FILE << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
