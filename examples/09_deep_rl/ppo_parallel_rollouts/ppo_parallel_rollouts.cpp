// Proximal policy optimisation on CartPole, learning from several episodes at once.
//
// PPOSolver learns from one episode at a time. ParallelPPOSolver plays an episode in each of the
// copies of a ParallelEnvs at the same time, with one call of the policy for all of them at every
// step, and learns from all the episodes together. The example trains both on the same number of
// episodes of bitrl's own CartPole, which needs no server, and then shows what stepping the copies
// on threads gains when an environment step takes time, as a physics engine's does.
//
// Usage: ppo_parallel_rollouts [--episodes N] [--copies N]
//
// Writes returns.csv, the mean return per episode of both trainings, to the working directory.

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/classic_control/cart_pole.h"
#include "bitrl/envs/parallel_envs.h"
#include "bitrl/maths/optimization/optimizer_type.h"
#include "bitrl/maths/optimization/pytorch_optimizer_factory.h"
#include "bitrl/maths/statistics/distributions/torch_categorical.h"
#include "bitrl/rl/algorithms/pg/parallel_ppo.h"
#include "bitrl/rl/algorithms/pg/ppo.h"
#include "bitrl/rl/algorithms/pg/ppo_config.h"
#include "bitrl/rl/trainers/rl_serial_agent_trainer.h"
#include "bitsim/utils/concurrency/thread_pool.h"
#include "bitsim/utils/io/print_utils.h"

#include <torch/torch.h>

#include <algorithm>
#include <any>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace ppo_parallel_rollouts
{

using bitrl::real_t;
using bitrl::torch_tensor_t;
using bitrl::uint_t;
using bitrl::envs::ParallelEnvs;
using bitrl::envs::classic_control::CartPole;
using bitrl::maths::stats::TorchCategorical;
namespace pg = bitrl::rl::algos::pg;
namespace optim = bitrl::maths::optim;
using steady = std::chrono::steady_clock;

const uint_t SEED = 42;
const uint_t MAX_STEPS = 500;
const real_t LEARNING_RATE = 0.001;

torch_tensor_t to_tensor(const std::vector<real_t> &state)
{
    return torch::tensor(state, torch::dtype(torch::kFloat32));
}

// The actor: the 4 numbers of the state in, the logits of the 2 actions out
struct ActorImpl : torch::nn::Module
{
    ActorImpl()
        : fc1(register_module("fc1", torch::nn::Linear(4, 64))),
          fc2(register_module("fc2", torch::nn::Linear(64, 64))),
          logits(register_module("logits", torch::nn::Linear(64, 2)))
    {
    }

    torch_tensor_t forward(torch_tensor_t states)
    {
        return logits->forward(torch::tanh(fc2->forward(torch::tanh(fc1->forward(states)))));
    }

    // one state, for PPOSolver
    std::tuple<uint_t, torch_tensor_t> act(const std::vector<real_t> &state)
    {
        TorchCategorical distribution(forward(to_tensor(state)), /*from logits*/ true);
        const auto action = distribution.sample();
        return {static_cast<uint_t>(action.item<int64_t>()),
                distribution.log_prob(action).detach()};
    }

    // a batch of states, a row each, for ParallelPPOSolver
    std::tuple<torch_tensor_t, torch_tensor_t> act_batch(torch_tensor_t states)
    {
        TorchCategorical distribution(forward(states), /*from logits*/ true);
        const auto actions = distribution.sample();
        return {actions, distribution.log_prob(actions).detach()};
    }

    std::tuple<torch_tensor_t, torch_tensor_t, torch_tensor_t> evaluate(torch_tensor_t states,
                                                                        torch_tensor_t actions)
    {
        const auto out = forward(states);
        TorchCategorical distribution(out, /*from logits*/ true);
        return {distribution.log_prob(actions.view({-1})), distribution.entropy().mean(), out};
    }

    uint_t greedy(const std::vector<real_t> &state)
    {
        torch::NoGradGuard no_grad;
        return static_cast<uint_t>(forward(to_tensor(state)).argmax().item<int64_t>());
    }

    torch::nn::Linear fc1;
    torch::nn::Linear fc2;
    torch::nn::Linear logits;
};
TORCH_MODULE(Actor);

// The critic: the state in, its value out
struct CriticImpl : torch::nn::Module
{
    CriticImpl()
        : fc1(register_module("fc1", torch::nn::Linear(4, 64))),
          fc2(register_module("fc2", torch::nn::Linear(64, 64))),
          value(register_module("value", torch::nn::Linear(64, 1)))
    {
    }

    torch_tensor_t forward(torch_tensor_t states)
    {
        return value->forward(torch::tanh(fc2->forward(torch::tanh(fc1->forward(states)))));
    }

    torch_tensor_t evaluate(const std::vector<real_t> &state)
    {
        torch::NoGradGuard no_grad;
        return forward(to_tensor(state));
    }

    torch::nn::Linear fc1;
    torch::nn::Linear fc2;
    torch::nn::Linear value;
};
TORCH_MODULE(Critic);

// CartPole whose every step also takes STEP_COST of computing, as a physics engine's step would
struct SlowCartPole
{
    typedef CartPole::time_step_type time_step_type;
    typedef CartPole::state_type state_type;
    typedef CartPole::action_type action_type;
    static constexpr real_t STEP_COST = 1.0e-3; // [s]

    void make(const std::string &version,
              const std::unordered_map<std::string, std::any> &make_options,
              const std::unordered_map<std::string, std::any> &reset_options)
    {
        env.make(version, make_options, reset_options);
    }
    time_step_type reset() { return env.reset(); }
    time_step_type step(const action_type &action)
    {
        const auto until = steady::now() + std::chrono::duration_cast<steady::duration>(
                                               std::chrono::duration<real_t>(STEP_COST));
        while (steady::now() < until)
        {
        }
        return env.step(action);
    }
    void close() { env.close(); }

    CartPole env;
};

pg::PPOConfig ppo_config()
{
    pg::PPOConfig config;
    config.max_itrs_per_episode = MAX_STEPS;
    config.gamma = 0.99;
    config.clip_epsilon = 0.2;
    config.max_passes_over_batch = 4;
    config.normalize_advantages = true;
    return config;
}

// An actor, a critic and their optimisers, from the same seed every time
struct Networks
{
    Actor actor;
    Critic critic;
    std::unique_ptr<torch::optim::Optimizer> actor_optimizer;
    std::unique_ptr<torch::optim::Optimizer> critic_optimizer;

    Networks()
    {
        torch::manual_seed(SEED);
        actor = Actor();
        critic = Critic();
        std::map<std::string, std::any> options{{"lr", std::any(LEARNING_RATE)}};
        const auto adam =
            optim::pytorch::build_pytorch_optimizer_options(optim::OptimizerType::ADAM, options);
        actor_optimizer =
            optim::pytorch::build_pytorch_optimizer(optim::OptimizerType::ADAM, *actor, *adam);
        critic_optimizer =
            optim::pytorch::build_pytorch_optimizer(optim::OptimizerType::ADAM, *critic, *adam);
    }
};

// The mean return of the greedy policy over 20 episodes it has not trained on
real_t greedy_return(Actor &actor)
{
    CartPole env;
    env.make("v1", {{"max_steps", MAX_STEPS}}, {});
    real_t total = 0.0;
    for (uint_t k = 0; k < 20; ++k)
    {
        auto step = env.reset(1000 + k);
        while (!step.last())
        {
            step = env.step(actor->greedy(step.observation()));
            total += step.reward();
        }
    }
    return total / 20.0;
}

real_t seconds(steady::duration d) { return std::chrono::duration<real_t>(d).count(); }

real_t tail_mean(const std::vector<real_t> &v)
{
    const auto n = std::max<std::size_t>(1, v.size() / 10);
    real_t sum = 0.0;
    for (auto i = v.size() - n; i < v.size(); ++i)
    {
        sum += v[i];
    }
    return sum / n;
}

struct Training
{
    std::vector<real_t> returns; // per episode
    real_t time{0.0};
    real_t greedy{0.0};
    uint_t n_steps{0};
};

// PPOSolver: an update after every episode
Training one_at_a_time(uint_t n_episodes)
{
    CartPole env;
    env.make("v1", {{"max_steps", MAX_STEPS}}, {{"seed", SEED}});
    Networks nets;
    auto config = ppo_config();
    config.n_episodes = n_episodes;
    typedef pg::PPOSolver<CartPole, Actor, Critic> solver_type;
    solver_type solver(config, nets.actor, nets.critic, nets.actor_optimizer,
                       nets.critic_optimizer);
    bitrl::rl::RLSerialAgentTrainer<CartPole, solver_type> trainer(
        {bitsim::consts::INVALID_ID, n_episodes, 1.0e-8}, solver);
    const auto start = steady::now();
    trainer.train(env);
    Training t;
    t.time = seconds(steady::now() - start);
    t.returns = solver.get_monitor().rewards;
    for (const auto r : t.returns)
    {
        t.n_steps += static_cast<uint_t>(r);
    }
    t.greedy = greedy_return(nets.actor);
    return t;
}

// ParallelPPOSolver: an update after every round of n_copies episodes
template <typename Env>
Training in_rounds(uint_t n_episodes, uint_t n_copies, bool threads, uint_t max_rounds = 0)
{
    ParallelEnvs<Env> envs(n_copies);
    envs.make("v1", {{"max_steps", MAX_STEPS}}, {{"seed", SEED}});
    envs.use_threads(threads);
    Networks nets;
    auto config = ppo_config();
    uint_t n_rounds = n_episodes / n_copies;
    if (max_rounds)
    {
        n_rounds = std::min(n_rounds, max_rounds);
    }
    config.n_episodes = n_rounds;
    typedef pg::ParallelPPOSolver<Env, Actor, Critic> solver_type;
    solver_type solver(config, nets.actor, nets.critic, nets.actor_optimizer,
                       nets.critic_optimizer);
    Training t;
    const auto start = steady::now();
    // the trainer's loop, by hand, to keep the return of every episode of every round
    solver.actions_before_training_begins(envs);
    for (uint_t round = 0; round < n_rounds; ++round)
    {
        const auto info = solver.on_training_episode(envs, round);
        t.returns.insert(t.returns.end(), solver.last_returns().begin(),
                         solver.last_returns().end());
        t.n_steps += info.episode_iterations;
    }
    t.time = seconds(steady::now() - start);
    t.greedy = greedy_return(nets.actor);
    return t;
}

void report(const std::string &name, const Training &t)
{
    std::cout << "  " << std::left << std::setw(26) << name << std::right << std::fixed
              << std::setprecision(1) << std::setw(8) << t.time << " s" << std::setw(10)
              << t.n_steps << std::setw(12) << tail_mean(t.returns) << std::setw(14) << t.greedy
              << "\n";
}

int run(uint_t n_episodes, uint_t n_copies)
{
    // the networks are small: libtorch's own threads cost more than they gain
    torch::set_num_threads(1);

    bitsim::utils::io::print_title("1. The same number of episodes, learned from one or " +
                                   std::to_string(n_copies) + " at a time");
    std::cout << "  " << n_episodes << " episodes of CartPole, at most " << MAX_STEPS
              << " steps each\n\n"
              << "  " << std::left << std::setw(26) << "" << std::right << std::setw(10) << "time"
              << std::setw(10) << "steps" << std::setw(12) << "last 10 %" << std::setw(14)
              << "greedy return"
              << "\n";
    const auto one = one_at_a_time(n_episodes);
    report("PPOSolver", one);
    const auto many = in_rounds<CartPole>(n_episodes, n_copies, false);
    report("ParallelPPOSolver", many);
    const auto threaded = in_rounds<CartPole>(n_episodes, n_copies, true);
    report("  and copies on threads", threaded);
    std::cout << "  the copies on threads learned the same: " << std::boolalpha
              << (threaded.returns == many.returns) << "\n\n";

    std::ofstream out("returns.csv");
    out << "episode,one_at_a_time,in_rounds\n";
    for (std::size_t e = 0; e < std::min(one.returns.size(), many.returns.size()); ++e)
    {
        out << e << "," << one.returns[e] << "," << many.returns[e] << "\n";
    }

    bitsim::utils::io::print_title("2. An environment whose step takes " +
                                   std::to_string(static_cast<int>(SlowCartPole::STEP_COST * 1e6)) +
                                   " microseconds");
    const uint_t ROUNDS = 3;
    const auto serial = in_rounds<SlowCartPole>(n_episodes, n_copies, false, ROUNDS);
    const auto parallel = in_rounds<SlowCartPole>(n_episodes, n_copies, true, ROUNDS);
    const auto threads = bitsim::utils::concurrency::default_thread_pool().size() + 1;
    std::cout << "  " << ROUNDS << " rounds of " << n_copies << " episodes, " << serial.n_steps
              << " steps\n"
              << "  the copies one after the other: " << std::setprecision(2) << serial.time
              << " s; on " << threads << " threads: " << parallel.time << " s, a speed-up of "
              << serial.time / parallel.time << "\n"
              << "  the same episodes: " << std::boolalpha << (serial.returns == parallel.returns)
              << "\n";
    return 0;
}

} // namespace ppo_parallel_rollouts

int main(int argc, char **argv)
{
    bitrl::uint_t n_episodes = 800;
    bitrl::uint_t n_copies = 8;
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string flag = argv[i];
        if (flag == "--episodes")
            n_episodes = std::strtoul(argv[i + 1], nullptr, 10);
        else if (flag == "--copies")
            n_copies = std::strtoul(argv[i + 1], nullptr, 10);
        else
        {
            std::cerr << "unknown option " << flag << "\n";
            return 1;
        }
    }
    return ppo_parallel_rollouts::run(n_episodes, n_copies);
}
