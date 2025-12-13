#include "bitrl/bitrl_types.h"
#include "bitrl/envs/gymnasium/classic_control/vector/acrobot_vec_env.h"
#include "bitrl/network/rest_rl_env_client.h"
#include "bitrl/bitrl_consts.h"

#include <iostream>
#include <string>
#include <unordered_map>
#include <any>
#include <vector>
#include <ios>


int main(){

	using namespace bitrl::envs::gymnasium;
	using bitrl::uint_t;
	using bitrl::network::RESTRLEnvClient;
	
	const std::string SERVER_URL = "http://0.0.0.0:8001/api";
    
	RESTRLEnvClient server(SERVER_URL, true);
	
	// Acrobot vector environment
	AcrobotV env(server);
	
	std::cout<<"Name: "<<env.name<<std::endl;
	std::cout<<"Number of actions: "<<env.n_actions()<<std::endl;
	
	std::unordered_map<std::string, std::any> options;
	options["num_envs"] = std::any(static_cast<uint_t>(3));

	std::unordered_map<std::string, std::any> reset_ops;
	reset_ops.insert({"seed", static_cast<uint_t>(42)});

	// make the environment
	env.make("v1", options, reset_ops);
	
	std::cout<<"Reseting the environment... "<<std::endl;
	auto time_step = env.reset();
	std::cout<<"Time step: "<<time_step<<std::endl;
	
	std::cout<<"Acting on the environment... "<<std::endl;
	// step in the environment
	std::vector<uint_t> actions(3, 0);
	actions[1] = 1;
	actions[2] = 2;
	time_step = env.step(actions);
	
	std::cout<<"Time step after action: "<<time_step<<std::endl;
	
	std::cout<<"Closing the environment... "<<std::endl;
	env.close();
	
	std::cout<<"Is active? "<<env.is_alive()<<std::noboolalpha <<std::endl;
    return 0;
}
