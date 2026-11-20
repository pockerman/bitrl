#include "cuberl/base/cuberl_types.h"
#include "cuberl/rl/algorithms/dp/iterative_policy_evaluation.h"
#include "cuberl/rl/trainers/rl_serial_agent_trainer.h"
#include "cuberl/rl/policies/uniform_discrete_policy.h"

#include "cuberl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "cuberl/network/rest_rl_env_client.h"

#include <iostream>

namespace rl_example_6
{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";

using cuberl::real_t;
using cuberl::uint_t;
using cuberl::rl::policies::UniformDiscretePolicy;
using cuberl::rl::algos::dp::IterativePolicyEvalutationSolver;
using cuberl::rl::algos::dp::IterativePolicyEvalConfig;
using cuberl::rl::RLSerialAgentTrainer;
using cuberl::rl::RLSerialTrainerConfig;
using cuberl::envs::gymnasium::FrozenLake;
using cuberl::network::RESTRLEnvClient;
typedef FrozenLake<4> env_type;

}

int main() {

    using namespace rl_example_6;
	
	RESTRLEnvClient server(SERVER_URL, true);

    // create the environment
    FrozenLake<4> env(server);

    std::cout<<"Environment URL: "<<env.get_url()<<std::endl;
    std::unordered_map<std::string, std::any> options;
	std::unordered_map<std::string, std::any> reset_options;
    std::cout<<"Creating the environment..."<<std::endl;
    env.make("v1", options, reset_options);
    env.reset();
    std::cout<<"Done..."<<std::endl;

    UniformDiscretePolicy policy(env.n_states(), env.n_actions());
    IterativePolicyEvalConfig config;
    config.tolerance = 1.0e-8;

    IterativePolicyEvalutationSolver<env_type, UniformDiscretePolicy> algorithm(config, policy);

    RLSerialTrainerConfig trainer_config = {10, 10000, 1.0e-8};

    RLSerialAgentTrainer<env_type,
                        IterativePolicyEvalutationSolver<env_type, UniformDiscretePolicy>> trainer(trainer_config, algorithm);

    auto info = trainer.train(env);
    std::cout<<info<<std::endl;

    // save the value function for plotting
    //algorithm.save("iterative_policy_evaluation_frozen_lake.csv");

   return 0;
}


