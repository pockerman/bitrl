\page bitrl_example_3 BitRL Example 3 Using Gymnasium environments (Part 2)

In this example we will see how to interact with  <a href="https://gymnasium.farama.org/index.html">Gymnasium</a> environments and
specifically how to create an interact with <a href="https://gymnasium.farama.org/environments/classic_control/cart_pole/">CartPole</a> environment.

As already mentioned in  \ref bitrl_example_1, Gymnasium-based environments are interacted over a REST-like API maintained here: <a href="https://github.com/pockerman/bitrl-rest-api">bitrl-envs-api</a>.
_bitrl_ itself implements classes that hide this interaction from the client code.
In general, environment classes in _bitrl_, have to implement the \ref bitrl::envs::EnvBase "bitrl::envs::EnvBase" API.

In this example we will use the \ref bitrl::envs::gymnasium::CartPole "bitrl::envs::gymnasium::CartPole"  
class. This is a template class, see the example below, that itself inherits from  \ref bitrl::envs::gymnasium::GymnasiumEnvBase "bitrl::envs::gymnasium::GymnasiumEnvBase"
class.

Below is the driver code. 

@code{.cpp}

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/gymnasium/classic_control/cart_pole_env.h"
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

} // namespace example

int main()
{

    using namespace example;

    const std::string SERVER_URL = "http://0.0.0.0:8001/api";

    RESTRLEnvClient server(SERVER_URL, true);

    std::cout << "Testing CartPole..." << std::endl;
    example::test_cart_pole(server);
    std::cout << "====================" << std::endl;

    return 0;
}
@endcode


In order to run the example you will need an instance of the <a href="https://github.com/pockerman/bitrl-rest-api">bitrl-envs-api</a> server running
on your machine listening at port 8001. Note the actual example also shows how to use \ref bitrl::envs::gymnasium::Pendulum "bitrl::envs::gymnasium::Pendulum",
\ref bitrl::envs::gymnasium::Acrobot "bitrl::envs::gymnasium::Acrobot" and \ref bitrl::envs::gymnasium::MountainCar "bitrl::envs::gymnasium::MountainCar"
environments.