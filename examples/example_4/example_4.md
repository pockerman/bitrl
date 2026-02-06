\page bitrl_example_4 BitRL Example 4 Using Gymnasium environments (Part 3)

This example follows \ref bitrl_example_1 and \ref bitrl_example_3 and shows how to use
\ref bitrl::envs::gymnasium::LunarLanderDiscreteEnv "bitrl::envs::gymnasium::LunarLanderDiscreteEnv"  and
\ref bitrl::envs::gymnasium::LunarLanderContinuousEnv "bitrl::envs::gymnasium::LunarLanderContinuousEnv".


@code{.cpp}

#include "bitrl/envs/gymnasium/box2d/lunar_lander_env.h"
#include "bitrl/network/rest_rl_env_client.h"

#include <iostream>
#include <unordered_map>
#include <vector>

namespace box2d_example
{
using namespace bitrl;
const std::string SERVER_URL = "http://0.0.0.0:8001/api";
using bitrl::real_t;
using bitrl::envs::gymnasium::LunarLanderContinuousEnv;
using bitrl::envs::gymnasium::LunarLanderDiscreteEnv;
} // namespace box2d_example

int main()
{
using namespace box2d_example;

    bitrl::network::RESTRLEnvClient server(SERVER_URL, true);

    std::unordered_map<std::string, std::any> options;
    options["wind_power"] = std::any(static_cast<bitrl::real_t>(10.0));
    options["enable_wind"] = std::any(static_cast<bool>(true));
    options["gravity"] = std::any(static_cast<bitrl::real_t>(-9.86));
    options["turbulence_power"] = std::any(static_cast<bitrl::real_t>(1.5));

    std::unordered_map<std::string, std::any> reset_ops;
    reset_ops.insert({"seed", static_cast<uint_t>(42)});

    {
        std::cout<<"Working with LunarLanderDiscreteEnv..."<<std::endl;
        std::cout << "Is environment registered: " << server.is_env_registered(LunarLanderDiscreteEnv::name)
              << std::endl;
        LunarLanderDiscreteEnv env(server);

        env.make("v3", options, reset_ops);

        std::cout<<"Is environment created? "<<env.is_created()<<std::endl;
        std::cout<<"Is environment alive? "<<env.is_alive()<<std::endl;
        std::cout<<"Number of valid actions? "<<env.n_actions()<<std::endl;

        auto time_step = env.reset();
        std::cout<<"Time step: "<<time_step<<std::endl;

        time_step = env.step(1);
        std::cout<<"Time step: "<<time_step<<std::endl;
        env.close();

    }
    {
        std::cout << "Working with LunarLanderContinuousEnv..." << std::endl;

        LunarLanderContinuousEnv env(server);
        env.make("v3", options, reset_ops);

        std::cout << "Is environment created? " << env.is_created() << std::endl;
        std::cout << "Is environment alive? " << env.is_alive() << std::endl;
        std::cout << "Action space size " << env.n_actions() << std::endl;
        std::cout << "Environment URI: " << server.get_uri(env.env_name()) << std::endl;

        auto time_step = env.reset();
        std::cout << "Time step: " << time_step << std::endl;

        std::vector<real_t> action = {0.8, 0.9};
        time_step = env.step(action);
        std::cout << "Time step: " << time_step << std::endl;
        env.close();
    }

    return 0;
}

@endcode