#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"
#include "bitrl/envs/connect2/connect2_env.h"
#include "bitrl/utils/io/io_utils.h"

#include <any>
#include <ios>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

int main()
{

    using namespace bitrl;
    using namespace bitrl::envs::connect2;
    using bitrl::uint_t;

    Connect2 env;

    std::cout << "Name: " << env.name << std::endl;
    std::cout << "Number of actions: " << env.n_actions() << std::endl;

    std::unordered_map<std::string, std::any> options;

    // make the environment
    env.make("v1", options, options);

    std::cout << "Is active? " << env.is_created() << std::noboolalpha << std::endl;

    std::cout << "Reseting the environment... " << std::endl;
    auto time_step = env.reset();

    std::cout << "Time step after reset: " << std::endl;
    std::cout << time_step << std::endl;

    std::cout << "Acting on the environment... " << std::endl;
    time_step = env.step(0);
    std::cout << "Observation vector..." << time_step.observation() << std::endl;

    // what are the valid moves remaining?
    auto val_moves = time_step.template get_extra<std::vector<int>>("valid_moves");

    std::cout << "Valid moves remaining..." << val_moves << std::endl;

    try
    {
        // invalid action
        env.step(5);
    }
    catch (std::logic_error &e)
    {
        std::cout << e.what() << std::endl;
    }

    // trying to step again on the same spot
    //  is an error
    try
    {
        env.step(0);
    }
    catch (std::logic_error &e)
    {
        std::cout << e.what() << std::endl;
    }

    std::cout << "Winning the game..." << std::endl;
    time_step = env.step(1);

    // this means that we have won
    std::cout << time_step << std::endl;

    time_step = env.reset();

    std::cout << "Starting the game..." << std::endl;
    std::cout << time_step.observation() << std::endl;
    std::cout << "Player 1 moves..." << std::endl;

    time_step = env.move(1, 2);
    std::cout << time_step.observation() << std::endl;
    std::cout << "Player 2 moves..." << std::endl;
    time_step = env.move(2, 3);
    std::cout << time_step.observation() << std::endl;
    std::cout << "Player 1 moves..." << std::endl;
    time_step = env.move(1, 1);
    std::cout << time_step.observation() << std::endl;

    std::cout << "Closing the environment... " << std::endl;
    env.close();

    return 0;
}
