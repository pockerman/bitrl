// Two agents learn to cross a corridor one cell wide from opposite ends, with independent
// Q-learning. Each agent learns its own Q-table; one team sees only its own position, the other
// also sees where the other agent is. Every team is trained with many seeds, all of them at the
// same time on the threads of bitsim's thread pool.
//
// Usage: multi_agent_grid_world [--episodes N] [--seeds N]
//
// Writes returns.csv (the mean return of the team per episode, for both observations), and
// layout.txt and episode.csv (the map, and the greedy episode of a team that sees both agents) to
// the working directory.
// examples/tools/multi_agent_gif.py turns episode.csv into a GIF.

#include "bitrl/envs/multi_agent/multi_agent_grid_world.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/multi_agent/independent_q_learning.h"
#include "bitrl/rl/multi_agent/multi_agent_trainer.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/concurrency/thread_pool.h"
#include "bitsim/utils/io/print_utils.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace multi_agent_grid_world
{

using bitrl::real_t;
using bitrl::uint_t;
using namespace bitrl::envs::multi_agent;
using namespace bitrl::rl::multi_agent;
using bitsim::utils::concurrency::parallel_map;
using bitsim::utils::concurrency::ThreadPool;

// What one training gave
struct Run
{
    std::vector<real_t> team_returns; // per episode
    bool solved{false};               // the greedy team reached both goals
    uint_t greedy_steps{0};
    uint_t greedy_collisions{0};
    IndependentQLearning learner;
};

MultiAgentGridWorld make_env(GridObservation observation)
{
    MultiAgentGridWorld env;
    env.make("v1", {{"observation", observation}, {"shared_reward", true}}, {});
    return env;
}

// Train one team from scratch; every call has its own environment and learner, so that many
// can run at once
Run train(GridObservation observation, uint_t seed, uint_t n_episodes)
{
    auto env = make_env(observation);
    IndependentQLearningConfig config;
    config.seed = seed;
    config.eta = 0.2;
    config.eps_decay = 5.0 / n_episodes;
    IndependentQLearning learner(env.n_agents(), env.n_states(), env.n_actions(), config);
    MultiAgentTrainer<MultiAgentGridWorld, IndependentQLearning> trainer({n_episodes}, learner);
    const auto training = trainer.train(env);

    Run run{{}, false, 0, 0, learner};
    for (const auto &episode : training.episodes)
    {
        run.team_returns.push_back(episode.returns.front()); // shared: every agent has the same
    }
    auto greedy = learner.greedy();
    const auto result = run_episode(env, greedy);
    run.solved = result.all_done;
    run.greedy_steps = result.n_steps;
    run.greedy_collisions = result.n_collisions;
    return run;
}

// Train n_seeds teams of each observation on pool, run k is seed task_seed(1, k)
std::vector<Run> train_all(uint_t n_seeds, uint_t n_episodes, ThreadPool &pool)
{
    return parallel_map(
        2 * n_seeds,
        [=](std::size_t k)
        {
            const auto observation =
                k < n_seeds ? GridObservation::OWN_POSITION : GridObservation::JOINT_POSITIONS;
            return train(observation, bitsim::utils::concurrency::task_seed(1, k % n_seeds),
                         n_episodes);
        },
        pool);
}

void summarise(const std::string &name, const std::vector<Run> &runs)
{
    uint_t solved = 0, steps = 0, collisions = 0;
    real_t final_return = 0.0;
    for (const auto &run : runs)
    {
        if (run.solved)
        {
            ++solved;
            steps += run.greedy_steps;
        }
        collisions += run.greedy_collisions;
        // the mean return of the last tenth of the episodes
        const auto tail = std::max<std::size_t>(1, run.team_returns.size() / 10);
        real_t sum = 0.0;
        for (auto i = run.team_returns.size() - tail; i < run.team_returns.size(); ++i)
        {
            sum += run.team_returns[i];
        }
        final_return += sum / tail;
    }
    std::cout << std::setw(18) << name << std::setw(10) << solved << "/" << runs.size()
              << std::setw(14) << std::fixed << std::setprecision(1)
              << (solved ? static_cast<real_t>(steps) / solved : 0.0) << std::setw(16)
              << static_cast<real_t>(collisions) / runs.size() << std::setw(16)
              << std::setprecision(2) << final_return / runs.size() << "\n";
}

void write_returns(const std::vector<Run> &own, const std::vector<Run> &joint)
{
    std::ofstream out("returns.csv");
    out << "episode,own_position,joint_positions\n";
    for (uint_t e = 0; e < own.front().team_returns.size(); ++e)
    {
        real_t a = 0.0, b = 0.0;
        for (const auto &run : own)
        {
            a += run.team_returns[e];
        }
        for (const auto &run : joint)
        {
            b += run.team_returns[e];
        }
        out << e << "," << a / own.size() << "," << b / joint.size() << "\n";
    }
}

// Play the greedy team of run and print and record its steps
void show(const Run &run, GridObservation observation)
{
    auto env = make_env(observation);
    env.reset();
    std::ofstream("layout.txt") << env.render();
    std::ofstream out("episode.csv");
    out << "step,agent,row,col,done\n";
    uint_t k = 0;
    auto greedy = run.learner.greedy();
    run_episode(env, greedy,
                [&](const MultiAgentGridWorld::time_step_type &step)
                {
                    for (uint_t a = 0; a < env.n_agents(); ++a)
                    {
                        out << k << "," << env.agent_name(a) << "," << env.positions()[a].first
                            << "," << env.positions()[a].second << "," << step.done(a) << "\n";
                    }
                    if (k % 2 == 0 || step.last())
                    {
                        std::cout << "  step " << k << (step.last() ? ", the last" : "") << "\n";
                        std::cout << env.render() << "\n";
                    }
                    ++k;
                });
}

void run(uint_t n_episodes, uint_t n_seeds)
{
    bitsim::utils::io::print_title("1. The corridor");
    MultiAgentGridWorld env = make_env(GridObservation::OWN_POSITION);
    env.reset();
    std::cout << env.render()
              << "\nA goes to a, B to b. The shortest paths meet in the corridor, so "
              << "one agent has to wait.\n\n";

    bitsim::utils::io::print_title("2. Training, in parallel");
    // the same trainings, once on one thread and once on all of them
    ThreadPool one_thread(1);
    auto start = std::chrono::steady_clock::now();
    train_all(std::max<uint_t>(1, n_seeds / 4), n_episodes, one_thread);
    const std::chrono::duration<real_t> serial = std::chrono::steady_clock::now() - start;

    start = std::chrono::steady_clock::now();
    const auto runs =
        train_all(n_seeds, n_episodes, bitsim::utils::concurrency::default_thread_pool());
    const std::chrono::duration<real_t> parallel = std::chrono::steady_clock::now() - start;

    // the one-thread batch had a quarter of the seeds; scale it to compare
    const real_t serial_estimate =
        serial.count() * static_cast<real_t>(n_seeds) / std::max<uint_t>(1, n_seeds / 4);
    std::cout << "  " << 2 * n_seeds << " trainings of " << n_episodes << " episodes on "
              << bitsim::utils::concurrency::default_thread_pool().size()
              << " threads: " << std::fixed << std::setprecision(2) << parallel.count()
              << " s; on one thread "
              << "they would take about " << serial_estimate << " s, a speed-up of "
              << serial_estimate / parallel.count() << "\n\n";

    const std::vector<Run> own(runs.begin(), runs.begin() + n_seeds);
    const std::vector<Run> joint(runs.begin() + n_seeds, runs.end());
    bitsim::utils::io::print_title("3. What the greedy teams do");
    std::cout << "       observation    solved   mean steps   collisions   final return\n";
    summarise("own position", own);
    summarise("joint positions", joint);
    write_returns(own, joint);
    std::cout << "\n  The mean return per episode is in returns.csv\n\n";

    bitsim::utils::io::print_title("4. A team that sees both agents");
    const auto best =
        std::find_if(joint.begin(), joint.end(), [](const Run &r) { return r.solved; });
    if (best == joint.end())
    {
        std::cout << "  no team solved the corridor; train for more episodes\n";
        return;
    }
    show(*best, GridObservation::JOINT_POSITIONS);
    std::cout << "  The episode is in episode.csv\n";
}

} // namespace multi_agent_grid_world

int main(int argc, char **argv)
{
    bitrl::uint_t n_episodes = 4000;
    bitrl::uint_t n_seeds = 16;
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string flag = argv[i];
        if (flag == "--episodes")
        {
            n_episodes = std::strtoul(argv[i + 1], nullptr, 10);
        }
        else if (flag == "--seeds")
        {
            n_seeds = std::strtoul(argv[i + 1], nullptr, 10);
        }
    }
    multi_agent_grid_world::run(n_episodes, n_seeds);
    return 0;
}
