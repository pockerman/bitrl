#include "cuberl/base/cuberl_types.h"
#include "cuberl/rl/algorithms/dp/value_iteration.h"
#include "cuberl/rl/trainers/rl_serial_agent_trainer.h"

#include "bitrl/utils/io/csv_file_writer.h"
#include "cuberl/network/rest_rl_env_client.h"
#include "cuberl/envs/gymnasium/toy_text/frozen_lake_env.h"

#include <boost/log/trivial.hpp>
#include <tuple>

namespace example_14{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string SOLUTION_FILE = "value_iteration_frozen_lake_v1.csv";
const std::string REWARD_PER_ITR = "reward_per_itr.csv";
const std::string POLICY = "policy.csv";

using cuberl::real_t;
using cuberl::uint_t;
using cuberl::rl::algos::dp::ValueIteration;
using cuberl::rl::algos::dp::ValueIterationConfig;
using cuberl::rl::RLSerialAgentTrainer;
using cuberl::rl::RLSerialTrainerConfig;

using cuberl::envs::gymnasium::FrozenLake;
using cuberl::network::RESTRLEnvClient;

// the environment to use
typedef FrozenLake<4> env_type;

// value iteration solver
typedef  ValueIteration<env_type> solver_type;
						
}

int main() {

	BOOST_LOG_TRIVIAL(info)<<"Starting agent training";
	
    using namespace example_14;
	
	RESTRLEnvClient server(SERVER_URL, true);

    // create the environment
    env_type env(server);

    BOOST_LOG_TRIVIAL(info)<<"Creating environment...";
    std::unordered_map<std::string, std::any> options;

	options["is_slippery"] = std::any(false);

	 std::unordered_map<std::string, std::any> reset_options;
    env.make("v1", options, reset_options);
    env.reset();
    BOOST_LOG_TRIVIAL(info)<<"Done...";

    ValueIterationConfig config;
    config.gamma = 0.99;
    config.tolerance = 1.0e-8;

    solver_type algorithm(config);

    RLSerialTrainerConfig trainer_config = {10, 10000, 1.0e-8};

    RLSerialAgentTrainer<env_type, solver_type> trainer(trainer_config, algorithm);

    auto info_ = trainer.train(env);
	BOOST_LOG_TRIVIAL(info)<<info_;
	BOOST_LOG_TRIVIAL(info)<<"Saving solution to "<<SOLUTION_FILE;

    // save the value function into a csv file
    algorithm.save(SOLUTION_FILE);
	
	
	// save the reward the agent achieved per training epoch
	auto reward = trainer.episodes_total_rewards();
	
	bitrl::utils::io::CSVWriter csv_writer(REWARD_PER_ITR);
	csv_writer.open();
	
	csv_writer.write_column_names({"epoch", "reward"});
	
	auto epoch = static_cast<uint_t>(0);
	for(auto val: reward){
		
		std::tuple<uint_t, real_t> row = {epoch++, val};
		csv_writer.write_row(row);
	}
	
	csv_writer.close();
	
	// build the policy
	algorithm.build_policy(env).save(POLICY);
	
	BOOST_LOG_TRIVIAL(info)<<"Finished agent training";
    return 0;
}


