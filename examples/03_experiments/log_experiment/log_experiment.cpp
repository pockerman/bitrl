// Logging a reinforcement learning experiment
//
// An experiment is worth something only if it can be read back and compared
// later: what it ran with, what happened in every episode, and what came of
// it. This example runs an experiment, Q-learning on the cliff walk with two
// learning rates, and logs it in a directory of its own:
//
//     experiments/<name>_<date>_<time>/
//         config.json            the settings, written before training
//         <run>/episodes.csv     the return and the iterations of every episode
//         <run>/policy.csv       the greedy policy the run ended with
//         summary.json           the result of every run, written at the end
//
// With --tensorboard URL it also sends the return and the iterations of every
// episode to Tensorboard, through the bitrl-rest-api server at URL, one run
// each, so that the runs can be compared there as they train.
//
// Usage: log_experiment [--tensorboard URL]

#include "cliff_walk.h"

#include "bitrl/base/bitrl_types.h"
#include "bitrl/network/tensorboard_server.h"
#include "bitrl/rl/algorithms/td/q_learning.h"
#include "bitrl/rl/policies/epsilon_greedy_policy.h"
#include "bitsim/extern/nlohmann/json/json.hpp"
#include "bitsim/utils/io/csv_file_writer.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace log_experiment
{

using bitrl::real_t;
using bitrl::uint_t;
using bitrl::rl::policies::EpsilonGreedyPolicy;
using examples::CliffWalk;
using json = nlohmann::json;
namespace fs = std::filesystem;

// The experiment: one run for every learning rate
struct Config
{
    std::string name{"q_learning_cliff_walk"};
    std::vector<real_t> learning_rates{0.1, 0.5};
    uint_t n_episodes{500};
    uint_t max_steps{100};
    real_t gamma{1.0};
    real_t epsilon{0.1};
    uint_t seed{42};
};

json to_json(const Config &c)
{
    return {{"name", c.name},
            {"algorithm", "Q-learning"},
            {"environment", "cliff walk, 4 x 12"},
            {"learning_rates", c.learning_rates},
            {"n_episodes", c.n_episodes},
            {"max_steps", c.max_steps},
            {"gamma", c.gamma},
            {"epsilon", c.epsilon},
            {"seed", c.seed}};
}

// experiments/<name>_<date>_<time>
fs::path experiment_dir(const Config &c)
{
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::ostringstream name;
    name << c.name << "_" << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    return fs::path("experiments") / name.str();
}

// eta_0p1 for 0.1: bitsim's file writers take a dot anywhere in the path for
// the start of the file's suffix
std::string run_name(real_t learning_rate)
{
    std::ostringstream name;
    name << "eta_" << learning_rate;
    auto s = name.str();
    std::replace(s.begin(), s.end(), '.', 'p');
    return s;
}

// The mean of the last n values
real_t mean_of_last(const std::vector<real_t> &values, std::size_t n)
{
    n = std::min(n, values.size());
    return std::accumulate(values.end() - n, values.end(), 0.0) / static_cast<real_t>(n);
}

// One run: train, logging every episode, and return what it came to
json run(const Config &c, real_t learning_rate, const fs::path &dir,
         bitrl::network::TensorboardServer *tensorboard)
{
    fs::create_directories(dir);
    bitsim::utils::io::CSVWriter episodes((dir / "episodes.csv").string(), ',');
    episodes.open();
    episodes.write_column_names({"episode", "return", "iterations"});

    if (tensorboard)
    {
        tensorboard->init(fs::absolute(dir / "tensorboard").string());
    }

    CliffWalk env;
    bitrl::rl::algos::td::QLearningConfig config;
    config.n_episodes = c.n_episodes;
    config.max_num_iterations_per_episode = c.max_steps;
    config.tolerance = 1.0e-8;
    config.gamma = c.gamma;
    config.eta = learning_rate;
    // the return of every episode, not the mean reward of its steps
    config.average_episode_reward = false;
    bitrl::rl::algos::td::QLearningSolver<CliffWalk, EpsilonGreedyPolicy> solver(
        config, EpsilonGreedyPolicy(c.epsilon, c.seed));

    std::vector<real_t> returns;
    solver.actions_before_training_begins(env);
    for (uint_t episode = 0; episode < c.n_episodes; ++episode)
    {
        solver.actions_before_episode_begins(env, episode);
        const auto info = solver.on_training_episode(env, episode);
        solver.actions_after_episode_ends(env, episode, info);

        const real_t episode_return = info.episode_reward;
        returns.push_back(episode_return);
        episodes.write_row(std::make_tuple(episode, episode_return, info.episode_iterations));
        if (tensorboard)
        {
            tensorboard->add_scalar("return", episode_return, episode);
            tensorboard->add_scalar("iterations", info.episode_iterations, episode);
        }
    }
    solver.actions_after_training_ends(env);
    episodes.close();
    if (tensorboard)
    {
        tensorboard->close();
    }

    const auto policy = solver.build_policy();
    policy.save((dir / "policy.csv").string());

    return {{"learning_rate", learning_rate},
            {"best_return", *std::max_element(returns.begin(), returns.end())},
            {"mean_return_last_100", mean_of_last(returns, 100)},
            {"episodes", (dir / "episodes.csv").string()},
            {"policy", (dir / "policy.csv").string()}};
}

} // namespace log_experiment

int main(int argc, char **argv)
{
    using namespace log_experiment;

    std::unique_ptr<bitrl::network::TensorboardServer> tensorboard;
    if (argc == 3 && std::string(argv[1]) == "--tensorboard")
    {
        tensorboard = std::make_unique<bitrl::network::TensorboardServer>(argv[2]);
    }
    else if (argc != 1)
    {
        std::cout << "Usage: log_experiment [--tensorboard URL]\n"
                  << "  --tensorboard URL   also log to Tensorboard through the bitrl-rest-api\n"
                  << "                      server at URL, for example http://0.0.0.0:8001"
                  << std::endl;
        return 1;
    }

    try
    {
        const Config config;
        const auto dir = experiment_dir(config);
        fs::create_directories(dir);

        // the settings first, so that even a run that fails says what it ran with
        std::ofstream(dir / "config.json") << std::setw(2) << to_json(config) << "\n";

        const auto wall_start = std::chrono::steady_clock::now();
        json runs = json::array();
        for (const auto eta : config.learning_rates)
        {
            const auto result = run(config, eta, dir / run_name(eta), tensorboard.get());
            std::cout << run_name(eta) << ": mean return of the last 100 episodes "
                      << result["mean_return_last_100"].get<real_t>() << ", best "
                      << result["best_return"].get<real_t>() << "\n";
            runs.push_back(result);
        }
        const real_t wall =
            std::chrono::duration<real_t>(std::chrono::steady_clock::now() - wall_start).count();

        std::ofstream(dir / "summary.json")
            << std::setw(2) << json{{"runs", runs}, {"wall_time_s", wall}} << "\n";
        std::cout << "The experiment is in " << dir.string() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
