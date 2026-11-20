#include "cuberl/base/cuberl_types.h"
#include "cuberl/rl/algorithms/td/sarsa.h"
#include "cuberl/rl/policies/epsilon_greedy_policy.h"
#include "cuberl/rl/trainers/rl_serial_agent_trainer.h"
#include "cuberl/network/rest_rl_env_client.h"
#include "cuberl/envs/gymnasium/toy_text/cliff_world_env.h"

#include <boost/log/trivial.hpp>
#include <iostream>

namespace rl_example_9{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";
const std::string SOLUTION_FILE = "sarsa_cliff_walking_v0.csv";
const std::string REWARD_PER_ITR = "reward_per_itr.csv";
const std::string POLICY = "policy.csv";

using cuberl::real_t;
using cuberl::uint_t;
using cuberl::rl::policies::EpsilonGreedyPolicy;
using cuberl::rl::algos::td::SarsaSolver;
using cuberl::rl::algos::td::SarsaConfig;
using cuberl::rl::policies::EpsilonDecayOption;
using cuberl::rl::RLSerialAgentTrainer;
using cuberl::rl::RLSerialTrainerConfig;
using cuberl::network::RESTRLEnvClient;
typedef  cuberl::envs::gymnasium::CliffWorld env_type;


}

int main(){

	BOOST_LOG_TRIVIAL(info)<<"Starting agent training";
    using namespace rl_example_9;

    try{

		RESTRLEnvClient server(SERVER_URL, true);
		
        // create the environment
        env_type env(server);

        BOOST_LOG_TRIVIAL(info)<<"Creating environment...";
        std::unordered_map<std::string, std::any> options;
    	std::unordered_map<std::string, std::any> reset_options;
        env.make("v0", options, reset_options);
        env.reset();
        BOOST_LOG_TRIVIAL(info)<<"Done...";

        BOOST_LOG_TRIVIAL(info)<<"Number of states="<<env.n_states();
        BOOST_LOG_TRIVIAL(info)<<"Number of actions="<<env.n_actions();

        // create an e-greedy policy. Use the number 
		// of actions as a seed. Use a constant epsilon
        EpsilonGreedyPolicy policy(0.1, env.n_actions(), 
		                           EpsilonDecayOption::NONE);

        SarsaConfig sarsa_config;
        sarsa_config.gamma = 1.0;
        sarsa_config.eta = 0.5;
        sarsa_config.tolerance = 1.0e-8;
        sarsa_config.max_num_iterations_per_episode = 100;
        sarsa_config.path = SOLUTION_FILE;

        SarsaSolver<env_type, EpsilonGreedyPolicy> algorithm(sarsa_config, policy);

        RLSerialTrainerConfig trainer_config = {10, 2000, 1.0e-8};

        RLSerialAgentTrainer<env_type,
                             SarsaSolver<env_type,
                             EpsilonGreedyPolicy>> trainer(trainer_config, algorithm);

        auto info = trainer.train(env);
        BOOST_LOG_TRIVIAL(info)<<"Training info..."<<info;
		BOOST_LOG_TRIVIAL(info)<<"Finished agent training";
		
		// save the reward the agent achieved per training epoch
		auto reward = trainer.episodes_total_rewards();
		auto iterations = trainer.n_itrs_per_episode();
	
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
		algorithm.build_policy().save(POLICY);

    }
    catch(std::exception& e){
        std::cout<<e.what()<<std::endl;
    }
    catch(...){

        std::cout<<"Unknown exception occured"<<std::endl;
    }

    return 0;
}
