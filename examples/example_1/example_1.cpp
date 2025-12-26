#include "bitrl/bitrl_types.h"
#include "bitrl/envs/gymnasium/toy_text/black_jack_env.h"
#include "bitrl/envs/gymnasium/toy_text/cliff_world_env.h"
#include "bitrl/envs/gymnasium/toy_text/frozen_lake_env.h"
#include "bitrl/envs/gymnasium/toy_text/taxi_env.h"
#include "bitrl/network/rest_rl_env_client.h"

#include <any>
#include <iostream>
#include <string>
#include <unordered_map>

namespace example_1
{
using namespace bitrl;

const std::string SERVER_URL = "http://0.0.0.0:8001/api";

using bitrl::envs::gymnasium::BlackJack;
using bitrl::envs::gymnasium::CliffWorld;
using bitrl::envs::gymnasium::FrozenLake;
using bitrl::envs::gymnasium::Taxi;
using bitrl::network::RESTRLEnvClient;

void test_frozen_lake(RESTRLEnvClient &server)
{

    // the environment is not registered with the server
    std::cout << "Is environment registered: " << server.is_env_registered(FrozenLake<4>::name)
              << std::endl;

    // when the environment is created we register it with the REST client
    FrozenLake<4> env(server);

    // environment name can also be accessed via env.env_name()
    std::cout << "Is environment registered: " << server.is_env_registered(env.env_name())
              << std::endl;
    std::cout << "Environment URL: " << env.get_url() << std::endl;

    // make the environment we pass both make options
    // and reset options
    std::unordered_map<std::string, std::any> make_ops;
    make_ops.insert({"is_slippery", false});

    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});
    env.make("v1", make_ops, reset_ops);

    // query the environemnt version
    std::cout << "Environment version: " << env.version() << std::endl;

    // once the env is created we can get it's id
    std::cout << "Environment idx is: " << env.idx() << std::endl;

    // the create flag should be true
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
    auto new_time_step = env.step(2);
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

void test_taxi(RESTRLEnvClient &server)
{

    // the environment is not registered with the server
    std::cout << "Is environment registered: " << server.is_env_registered(FrozenLake<4>::name)
              << std::endl;

    Taxi env(server);

    std::cout << "Environment URL: " << env.get_url() << std::endl;

    // make the environment
    std::unordered_map<std::string, std::any> make_ops;
    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});
    env.make("v3", make_ops, reset_ops);

    // query the environemnt version
    std::cout << "Environment version: " << env.version() << std::endl;

    // once the env is created we can get it's id
    std::cout << "Environment idx is: " << env.idx() << std::endl;

    // the create flag should be true
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
    // move RIGHT
    auto new_time_step = env.step(2);
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

    // close the environment
    env.close();
}

void test_black_jack(RESTRLEnvClient &server)
{

    BlackJack env(server);
    std::unordered_map<std::string, std::any> options;
    options["natural"] = true;

    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});

    std::cout << "Environment created..." << std::endl;
    env.make("v1", options, reset_ops);

    std::cout << "Environment reset..." << std::endl;
    auto state = env.reset();

    std::cout << "Environment step..." << std::endl;

    // 0 = HIT
    // 1 = STICK
    env.step(0);
    env.step(1);

    // close the environment
    env.close();
}

void test_cliff_world(RESTRLEnvClient &server)
{

    CliffWorld env(server);

    std::cout << "Environment URL: " << env.get_url() << std::endl;

    // make the environment
    std::unordered_map<std::string, std::any> options;
    options["max_episode_steps"] = std::any(static_cast<bitrl::uint_t>(10));
    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});
    env.make("v1", options, reset_ops);

    // query the environemnt version
    std::cout << "Environment version: " << env.version() << std::endl;

    // once the env is created we can get it's id
    std::cout << "Environment idx is: " << env.idx() << std::endl;

    // the create flag should be true
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
    // 0 = UP
    auto new_time_step = env.step(0);

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

    // close the environment
    env.close();
}

} // namespace example_1

int main()
{

    using namespace example_1;

    RESTRLEnvClient server(SERVER_URL, false);

    std::cout << "Testing FrozenLake..." << std::endl;
    example_1::test_frozen_lake(server);
    std::cout << "====================" << std::endl;
    std::cout << "Testing Taxi..." << std::endl;
    example_1::test_taxi(server);
    std::cout << "====================" << std::endl;
    std::cout << "Testing BlackJack..." << std::endl;
    example_1::test_black_jack(server);
    std::cout << "====================" << std::endl;
    std::cout << "Testing CliffWorld..." << std::endl;
    example_1::test_cliff_world(server);
    std::cout << "====================" << std::endl;
    return 0;
}
