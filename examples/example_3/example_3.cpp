
#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/gymnasium/classic_control/acrobot_env.h"
#include "bitrl/envs/gymnasium/classic_control/cart_pole_env.h"
#include "bitrl/envs/gymnasium/classic_control/pendulum_env.h"
#include "bitrl/envs/gymnasium/classic_control/mountain_car_env.h"
#include "bitrl/network/rest_rl_env_client.h"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <iostream>
#include <random>
#include <string>
#include <unordered_map>

namespace example
{
using namespace bitrl;
using namespace bitrl::envs::gymnasium;
using bitrl::network::RESTRLEnvClient;

void test_cart_pole(RESTRLEnvClient &server)
{
    std::cout << "Is environment registered: " << server.is_env_registered(CartPole::name)
              << std::endl;

    // create the environment
    CartPole env(server);

    std::cout << "Name: " << env.name << std::endl;
    std::cout << "Number of actions: " << env.n_actions() << std::endl;

    // make the environment
    std::unordered_map<std::string, std::any> options;
    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});
    env.make("v1", options, reset_ops);

    auto time_step = env.reset();
    std::cout << "Time step: " << time_step << std::endl;

    // step in the environment
    time_step = env.step(0);
    std::cout << "Time step after action: " << time_step << std::endl;

    env.close();
}

void test_acrobot(RESTRLEnvClient &server)
{
    std::cout << "Is environment registered: " << server.is_env_registered(Acrobot::name)
              << std::endl;

    // create the environment
    Acrobot env(server);

    std::cout << "Name: " << env.name << std::endl;
    std::cout << "Number of actions: " << env.n_actions() << std::endl;

    // make the environment
    std::unordered_map<std::string, std::any> options;
    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});
    env.make("v1", options, reset_ops);

    auto time_step = env.reset();
    std::cout << "Time step: " << time_step << std::endl;

    // step in the environment
    time_step = env.step(0);
    std::cout << "Time step after action: " << time_step << std::endl;

    env.close();
}

void test_pendulum(RESTRLEnvClient &server)
{
    std::cout << "Is environment registered: " << server.is_env_registered(Pendulum::name)
              << std::endl;

    // create the environment
    Pendulum env(server);

    std::cout << "Name: " << env.name << std::endl;
    std::cout << "Number of actions: " << env.n_actions() << std::endl;

    // make the environment
    std::unordered_map<std::string, std::any> options;
    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});
    env.make("v1", options, reset_ops);

    auto time_step = env.reset();
    std::cout << "Time step: " << time_step << std::endl;

    // step in the environment
    time_step = env.step(0.0);
    std::cout << "Time step after action: " << time_step << std::endl;

    env.close();
}

void test_mountain_car(RESTRLEnvClient &server)
{
    std::cout << "Is environment registered: " << server.is_env_registered(MountainCar::name)
              << std::endl;

    // create the environment
    MountainCar env(server);

    std::cout << "Name: " << env.name << std::endl;
    std::cout << "Number of actions: " << env.n_actions() << std::endl;

    // make the environment
    std::unordered_map<std::string, std::any> options;
    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});
    env.make("v0", options, reset_ops);

    auto time_step = env.reset();
    std::cout << "Time step: " << time_step << std::endl;

    // step in the environment
    time_step = env.step(0.0);
    std::cout << "Time step after action: " << time_step << std::endl;

    env.close();
}

} // namespace example

int main()
{

    using namespace example;

    const std::string SERVER_URL = "http://0.0.0.0:8001/api";

    RESTRLEnvClient server(SERVER_URL, true);

    std::cout << "Testing CartPole..." << std::endl;
    example::test_cart_pole(server);
    std::cout << "====================" << std::endl;
    std::cout << "Testing Pendulum..." << std::endl;
    example::test_pendulum(server);
    std::cout << "====================" << std::endl;
    std::cout << "Testing Acrobot..." << std::endl;
    example::test_acrobot(server);
    std::cout << "====================" << std::endl;
    std::cout<<"Testing MountainCar..."<<std::endl;
    example::test_mountain_car(server);
    std::cout<<"===================="<<std::endl;
    return 0;

    return 0;
}
