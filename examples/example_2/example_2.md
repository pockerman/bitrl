\page bitrl_example_2 BitRL Example 2 Using GymWalk Environment

The \ref bitrl::envs::gdrl::GymWalk "bitrl::envs::gdrl::GymWalk" is an implementation of the
environment from https://github.com/mimoralea/gym-walk. Just like with \ref bitrl_example_1, you will need
an instance of the  <a href="https://github.com/pockerman/bitrl-rest-api">bitrl-envs-api</a> server running in order 
to actually use the environment. Other than that there isn't much to say for this example.
Below is the driver code.

@code{.cpp}
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/gdrl/gym_walk.h"
#include "bitrl/network/rest_rl_env_client.h"

#include <any>
#include <iostream>
#include <string>
#include <unordered_map>

namespace example_1
{
using namespace bitrl;

const std::string SERVER_URL = "http://0.0.0.0:8001/api";

using bitrl::envs::gdrl::GymWalk;
using bitrl::network::RESTRLEnvClient;

void test_gymwalk(RESTRLEnvClient &server)
{
// the environment is not registered with the server
std::cout << "Is environment registered: " << server.is_env_registered(GymWalk<4>::name)
<< std::endl;

    // when the environment is created we register it with the REST client
    GymWalk<4> env(server);

    // environment name can also be accessed via env.env_name()
    std::cout << "Is environment registered: " << server.is_env_registered(env.env_name())
              << std::endl;
    std::cout << "Environment URL: " << env.get_url() << std::endl;

    // make the environment we pass both make options
    // and reset options
    std::unordered_map<std::string, std::any> make_ops;
    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});
    env.make("v1", make_ops, reset_ops);

    // query the environemnt version
    std::cout << "Environment version: " << env.version() << std::endl;

    // once the env is created we can get it's id
    std::cout << "Environment idx is: " << env.idx() << std::endl;

    // the created flag should be true
    std::cout << "Is environment created? " << env.is_created() << std::endl;

    // environment should be alive on the server
    std::cout << "Is environment alive? " << env.is_alive() << std::endl;

    // FrozenLake is a discrete state-action env so we can
    // query number of actions and states
    std::cout << "Number of valid actions? " << env.n_actions() << std::endl;
    std::cout << "Number of states? " << env.n_states() << std::endl;

    // reset the environment
    auto time_step = env.reset();

    std::cout << "Reward on reset: " << time_step.reward() << std::endl;
    std::cout << "Observation on reset: " << time_step.observation() << std::endl;
    std::cout << "Is terminal state: " << time_step.done() << std::endl;

    //...print the time_step
    std::cout << time_step << std::endl;

    // take an action in the environment
    // 2 = RIGHT
    auto new_time_step = env.step(1);
    std::cout << new_time_step << std::endl;

    // get the dynamics of the environment for the given state and action
    auto state = 0;
    auto action = 1;
    auto dynamics = env.p(state, action);

    std::cout << "Dynamics for state=" << state << " and action=" << action << std::endl;
    for (auto item : dynamics)
    {
        std::cout << std::get<0>(item) << std::endl;
        std::cout << std::get<1>(item) << std::endl;
        std::cout << std::get<2>(item) << std::endl;
        std::cout << std::get<3>(item) << std::endl;
    }

    // discrete action environments can sample
    // actions
    action = env.sample_action();
    std::cout << "Action sampled: " << action << std::endl;

    new_time_step = env.step(action);
    std::cout << new_time_step << std::endl;

    // close the environment
    env.close();
}

} // namespace example_1

int main()
{

    using namespace example_1;

    RESTRLEnvClient server(SERVER_URL, false);

    std::cout << "Testing GymWalk..." << std::endl;
    example_1::test_gymwalk(server);
    std::cout << "====================" << std::endl;
    return 0;
}

@endcode
