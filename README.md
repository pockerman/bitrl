[![Build bitrl](https://github.com/pockerman/bitrl/actions/workflows/build.yml/badge.svg)](https://github.com/pockerman/bitrl/actions/workflows/build.yml)
# bitrl

```bitrl``` is an effort to provide implementations and wrappers of environments suitable for training reinforcement learning agents
using  C++.  The documentation for the library can be found <a href="https://pockerman.github.io/bitrl/index.html">here</a>.

The following is an example how to use the 
``FrozenLake``   environment from <a href="https://github.com/Farama-Foundation/Gymnasium/tree/main">Gymnasium</a>.

```

#include "bitrl/bitrl_types.h"
#include "bitrl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "bitrl/network/rest_rl_env_client.h"

#include <iostream>
#include <string>
#include <unordered_map>
#include <any>

namespace example_1{

const std::string SERVER_URL = "http://0.0.0.0:8001/api";

using bitrl::envs::gymnasium::FrozenLake;
using bitrl::envs::RESTApiServerWrapper;

void test_frozen_lake(const RESTApiServerWrapper& server){

    FrozenLake<4> env(server);

    std::cout<<"Environame URL: "<<env.get_url()<<std::endl;

    // make the environment
    std::unordered_map<std::string, std::any> make_ops;
    make_ops.insert({"is_slippery", false});

	std::unordered_map<std::string, std::any> reset_ops;
	reset_ops.insert({"seed", static_cast<uint_t>(42)});
    env.make("v1", make_ops, reset_ops);

    std::cout<<"Is environment created? "<<env.is_created()<<std::endl;
    std::cout<<"Is environment alive? "<<env.is_alive()<<std::endl;
    std::cout<<"Number of valid actions? "<<env.n_actions()<<std::endl;
    std::cout<<"Number of states? "<<env.n_states()<<std::endl;
	std::cout<<"Env idx: "<<env.idx()<<std::endl;

    // reset the environment
    auto time_step = env.reset();

    std::cout<<"Reward on reset: "<<time_step.reward()<<std::endl;
    std::cout<<"Observation on reset: "<<time_step.observation()<<std::endl;
    std::cout<<"Is terminal state: "<<time_step.done()<<std::endl;
	std::cout<<"Env idx: "<<env.idx()<<std::endl;

    //...print the time_step
    std::cout<<time_step<<std::endl;

    // take an action in the environment
	// 2 = RIGHT
    auto new_time_step = env.step(2);

    std::cout<<new_time_step<<std::endl;

    // get the dynamics of the environment for the given state and action
    auto state = 0;
    auto action = 1;
    auto dynamics = env.p(state, action);

    std::cout<<"Dynamics for state="<<state<<" and action="<<action<<std::endl;

    for(auto item:dynamics){
        std::cout<<std::get<0>(item)<<std::endl;
        std::cout<<std::get<1>(item)<<std::endl;
        std::cout<<std::get<2>(item)<<std::endl;
        std::cout<<std::get<3>(item)<<std::endl;
    }
	
	action = env.sample_action();
	std::cout<<"Action sampled: "<<action<<std::endl;
	
	new_time_step = env.step(action);
    std::cout<<new_time_step<<std::endl;

    // close the environment
    env.close();
}
}


int main(){

	using namespace example_1;
	
	RESTApiServerWrapper server(SERVER_URL, true);

    std::cout<<"Testing FrozenLake..."<<std::endl;
    example_1::test_frozen_lake(server);
    std::cout<<"===================="<<std::endl;
  
    return 0;
}

```

Gymnasium environments exposed over a REST like API can be found at: <a href="https://github.com/pockerman/bitrl-rest-api">bitrl-rest-api</a>
Various RL algorithms using the environments can be found at <a href="https://github.com/pockerman/cuberl/tree/master">cuberl</a>.

Furthermore, there is some minimal support for working with Arduino UNO boards over USB or WiFi.
See also <a href="https://rlenvscpp.readthedocs.io/en/latest/working_with_webots.html">Working with Webots</a>
for how to integrate ```bitrl``` with <a href="https://cyberbotics.com/doc/guide/installing-webots">Webots</a>.

