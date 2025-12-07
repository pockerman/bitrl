
#include "bitrl/envs/gymnasium/classic_control/pendulum_env.h"
#include "../../src/bitrl/network/rest_rl_env_client.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/bitrl_consts.h"


#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <iostream>
#include <string>
#include <random>
#include <unordered_map>

 


int main(){

	using namespace bitrl::envs::gymnasium;
	using bitrl::envs::RESTRLEnvClient;
	
	const std::string SERVER_URL = "http://0.0.0.0:8001/api";
    
	RESTRLEnvClient server(SERVER_URL, true);
	
	Pendulum env(server);
	
	std::cout<<"Name: "<<env.name<<std::endl;
	std::cout<<"Number of actions: "<<env.n_actions()<<std::endl;
	
	// make the environment
	std::unordered_map<std::string, std::any> options;
	env.make("v1", options);
	
	auto time_step = env.reset();
	
	std::cout<<"Time step: "<<time_step<<std::endl;
	
	// step in the environment
	time_step = env.step(0.0);
	
	std::cout<<"Time step after action: "<<time_step<<std::endl;
	
	env.close();
	
	std::cout<<"Is active? "<<env.is_alive()<<std::endl;
	
    return 0;
}
