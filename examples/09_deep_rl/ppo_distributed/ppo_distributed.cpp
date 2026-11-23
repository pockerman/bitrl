// Proximal policy optimisation over several ranks that average their gradients.
//
// Every rank plays its own episodes of CartPole, in copies of its own, and computes the gradients
// of its own episodes; the ranks replace them with their mean, through libtorch's TCPStore, and
// take the same optimiser step. They start from rank 0's networks, so they stay alike: one
// network trained on the episodes of all the ranks. The ranks can be threads, processes or
// machines.
//
// Usage: ppo_distributed [--episodes N] [--ranks N] [--copies N]
//        ppo_distributed --rank R --ranks N [--episodes N] [--copies N] [--host H] [--port P]
//
// Without --rank, the example trains with N ranks as threads of this process, and compares with
// ParallelPPOSolver on the same episodes in one thread. With --rank, it is one rank of N
// processes: start one for every R from 0 to N - 1; rank 0 runs the store at --host:--port, and
// the others connect to it.

#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/classic_control/cart_pole.h"
#include "bitrl/envs/parallel_envs.h"
#include "bitrl/maths/optimization/optimizer_type.h"
#include "bitrl/maths/optimization/pytorch_optimizer_factory.h"
#include "bitrl/maths/statistics/distributions/torch_categorical.h"
#include "bitrl/rl/algorithms/pg/distributed_ppo.h"
#include "bitrl/rl/algorithms/pg/parallel_ppo.h"
#include "bitrl/rl/algorithms/pg/ppo_config.h"
#include "bitrl/rl/distributed/communicator.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/io/print_utils.h"

#include <ATen/CPUGeneratorImpl.h>
#include <torch/torch.h>

#include <algorithm>
#include <any>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace ppo_distributed
{

using bitrl::real_t;
using bitrl::torch_tensor_t;
using bitrl::uint_t;
using bitrl::envs::ParallelEnvs;
using bitrl::envs::classic_control::CartPole;
using bitrl::maths::stats::TorchCategorical;
using bitrl::rl::distributed::Communicator;
using bitrl::rl::distributed::TCPStoreCommunicator;
using bitsim::utils::concurrency::task_seed;
namespace pg = bitrl::rl::algos::pg;
namespace optim = bitrl::maths::optim;
using steady = std::chrono::steady_clock;

const uint_t SEED = 42;
const uint_t MAX_STEPS = 500;
const real_t LEARNING_RATE = 0.001;

// The actor: the logits of CartPole's 2 actions. It samples with a generator of its own, so that
// ranks that are threads of one process do not share libtorch's generator
struct ActorImpl : torch::nn::Module
{
    explicit ActorImpl(uint_t seed = SEED)
        : fc1(register_module("fc1", torch::nn::Linear(4, 64))),
          fc2(register_module("fc2", torch::nn::Linear(64, 64))),
          logits(register_module("logits", torch::nn::Linear(64, 2))),
          generator(at::make_generator<at::CPUGeneratorImpl>(seed))
    {
    }

    torch_tensor_t forward(torch_tensor_t states)
    {
        return logits->forward(torch::tanh(fc2->forward(torch::tanh(fc1->forward(states)))));
    }

    // a batch of states, a row each, for ParallelPPOSolver and DistributedPPOSolver
    std::tuple<torch_tensor_t, torch_tensor_t> act_batch(torch_tensor_t states)
    {
        const auto log_p = torch::log_softmax(forward(states), -1);
        const auto actions = torch::multinomial(log_p.exp(), 1, true, generator);
        return {actions.view({-1}), log_p.gather(1, actions).view({-1}).detach()};
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
        return static_cast<uint_t>(
            forward(torch::tensor(state, torch::dtype(torch::kFloat32))).argmax().item<int64_t>());
    }

    torch::nn::Linear fc1;
    torch::nn::Linear fc2;
    torch::nn::Linear logits;
    at::Generator generator;
};
TORCH_MODULE(Actor);

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

    torch::nn::Linear fc1;
    torch::nn::Linear fc2;
    torch::nn::Linear value;
};
TORCH_MODULE(Critic);

pg::PPOConfig ppo_config(uint_t n_rounds)
{
    pg::PPOConfig config;
    config.n_episodes = n_rounds;
    config.max_itrs_per_episode = MAX_STEPS;
    config.gamma = 0.99;
    config.clip_epsilon = 0.2;
    config.max_passes_over_batch = 4;
    config.normalize_advantages = true;
    return config;
}

// An actor, a critic and their optimisers; the networks of rank r start from a seed of their own,
// which matters only for rank 0, whose networks every rank takes
struct Networks
{
    Actor actor{nullptr};
    Critic critic{nullptr};
    std::unique_ptr<torch::optim::Optimizer> actor_optimizer;
    std::unique_ptr<torch::optim::Optimizer> critic_optimizer;

    explicit Networks(uint_t rank)
    {
        torch::manual_seed(task_seed(SEED, rank));
        actor = Actor(task_seed(SEED + 1, rank));
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

// What one rank got
struct Rank
{
    std::vector<real_t> returns; // the mean return of all the ranks' episodes, per round
    std::uint64_t checksum{0};
    real_t greedy{0.0};
    real_t time{0.0};
};

// One rank: its own copies of CartPole, seeded for the rank, and its own networks
Rank train_rank(Communicator &communicator, Networks &nets, uint_t n_rounds, uint_t n_copies)
{
    const uint_t rank = communicator.rank();
    ParallelEnvs<CartPole> envs(n_copies);
    envs.make("v1", {{"max_steps", MAX_STEPS}}, {{"seed", task_seed(SEED, rank)}});
    envs.use_threads(false); // a CartPole step is too quick for threads

    pg::DistributedPPOSolver<CartPole, Actor, Critic> solver(ppo_config(n_rounds), nets.actor,
                                                             nets.critic, nets.actor_optimizer,
                                                             nets.critic_optimizer, communicator);
    const auto start = steady::now();
    solver.actions_before_training_begins(envs);
    for (uint_t round = 0; round < n_rounds; ++round)
    {
        solver.on_training_episode(envs, round);
    }
    Rank r;
    r.time = seconds(steady::now() - start);
    r.returns = solver.global_returns();
    r.checksum = solver.checksum();
    r.greedy = greedy_return(nets.actor);
    return r;
}

// The same episodes in one thread: ParallelPPOSolver with as many copies as all the ranks have
Rank one_thread(uint_t n_rounds, uint_t n_copies)
{
    Networks nets(0);
    ParallelEnvs<CartPole> envs(n_copies);
    envs.make("v1", {{"max_steps", MAX_STEPS}}, {{"seed", SEED}});
    envs.use_threads(false);
    pg::ParallelPPOSolver<CartPole, Actor, Critic> solver(
        ppo_config(n_rounds), nets.actor, nets.critic, nets.actor_optimizer, nets.critic_optimizer);
    const auto start = steady::now();
    solver.actions_before_training_begins(envs);
    for (uint_t round = 0; round < n_rounds; ++round)
    {
        solver.on_training_episode(envs, round);
    }
    Rank r;
    r.time = seconds(steady::now() - start);
    r.returns = solver.get_monitor().rewards;
    r.greedy = greedy_return(nets.actor);
    return r;
}

void report(const std::string &name, const Rank &r)
{
    std::cout << "  " << std::left << std::setw(34) << name << std::right << std::fixed
              << std::setprecision(1) << std::setw(7) << r.time << " s" << std::setw(12)
              << tail_mean(r.returns) << std::setw(15) << r.greedy << "\n";
}

bool compare(const std::string &title, uint_t n_episodes, uint_t n_ranks, uint_t n_copies,
             uint_t port)
{
    const uint_t n_rounds = n_episodes / (n_ranks * n_copies);
    bitsim::utils::io::print_title(title + std::to_string(n_episodes) + " episodes of CartPole, " +
                                   std::to_string(n_ranks * n_copies) + " per update");
    std::cout << "  " << std::left << std::setw(34) << "" << std::right << std::setw(9) << "time"
              << std::setw(12) << "last 10 %" << std::setw(15) << "greedy return"
              << "\n";

    const auto single = one_thread(n_rounds, n_ranks * n_copies);
    report("ParallelPPOSolver, 1 thread", single);

    // the networks first, one after the other: they start from libtorch's generator, which the
    // threads of a process share
    std::vector<std::unique_ptr<Networks>> nets;
    for (uint_t r = 0; r < n_ranks; ++r)
    {
        nets.push_back(std::make_unique<Networks>(r));
    }
    std::vector<Rank> ranks(n_ranks);
    std::vector<std::thread> threads;
    const auto start = steady::now();
    for (uint_t r = 0; r < n_ranks; ++r)
    {
        threads.emplace_back(
            [&, r]
            {
                TCPStoreCommunicator communicator("127.0.0.1", port, r, n_ranks);
                ranks[r] = train_rank(communicator, *nets[r], n_rounds, n_copies);
            });
    }
    for (auto &t : threads)
    {
        t.join();
    }
    Rank all = ranks.front();
    all.time = seconds(steady::now() - start);
    report("DistributedPPOSolver, " + std::to_string(n_ranks) + " ranks of " +
               std::to_string(n_copies),
           all);

    bool same = true;
    for (const auto &r : ranks)
    {
        same = same && r.checksum == ranks.front().checksum;
    }
    std::cout << "\n  every rank ended with the same networks: " << std::boolalpha << same
              << " (checksum " << std::hex << ranks.front().checksum << std::dec << ")\n\n";
    return same;
}

int run_threads(uint_t n_episodes, uint_t n_ranks, uint_t n_copies, uint_t port)
{
    // the same, then with four times as many episodes per update, which is where the update
    // takes long enough to be worth sharing
    const bool a = compare("1. ", n_episodes, n_ranks, n_copies, port);
    const bool b = compare("2. ", 2 * n_episodes, n_ranks, 4 * n_copies, port + 1);
    return a && b ? 0 : 1;
}

int run_rank(uint_t rank, uint_t n_episodes, uint_t n_ranks, uint_t n_copies,
             const std::string &host, uint_t port)
{
    const uint_t n_rounds = n_episodes / (n_ranks * n_copies);
    std::cout << "  rank " << rank << " of " << n_ranks << ": waiting for the others at " << host
              << ":" << port << std::endl;
    Networks nets(rank);
    TCPStoreCommunicator communicator(host, port, rank, n_ranks);
    const auto r = train_rank(communicator, nets, n_rounds, n_copies);
    std::cout << "  rank " << rank << ": " << n_rounds << " rounds in " << std::fixed
              << std::setprecision(1) << r.time << " s; mean return of the last 10 % "
              << tail_mean(r.returns) << "; greedy return " << r.greedy << "; checksum " << std::hex
              << r.checksum << std::dec << std::endl;
    return 0;
}

} // namespace ppo_distributed

int main(int argc, char **argv)
{
    bitrl::uint_t n_episodes = 800, n_ranks = 4, n_copies = 2, port = 29531;
    bitrl::uint_t rank = 0;
    bool is_rank = false;
    std::string host = "127.0.0.1";
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string flag = argv[i], value = argv[i + 1];
        if (flag == "--episodes")
            n_episodes = std::strtoul(value.c_str(), nullptr, 10);
        else if (flag == "--ranks")
            n_ranks = std::strtoul(value.c_str(), nullptr, 10);
        else if (flag == "--copies")
            n_copies = std::strtoul(value.c_str(), nullptr, 10);
        else if (flag == "--rank")
        {
            rank = std::strtoul(value.c_str(), nullptr, 10);
            is_rank = true;
        }
        else if (flag == "--host")
            host = value;
        else if (flag == "--port")
            port = std::strtoul(value.c_str(), nullptr, 10);
        else
        {
            std::cerr << "unknown option " << flag << "\n";
            return 1;
        }
    }
    if (n_ranks == 0 || n_copies == 0 || n_episodes < n_ranks * n_copies)
    {
        std::cerr << "need --episodes of at least --ranks times --copies\n";
        return 1;
    }
    // the networks are small: libtorch's own threads cost more than they gain
    torch::set_num_threads(1);
    try
    {
        if (is_rank)
        {
            return ppo_distributed::run_rank(rank, n_episodes, n_ranks, n_copies, host, port);
        }
        return ppo_distributed::run_threads(n_episodes, n_ranks, n_copies, port);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
