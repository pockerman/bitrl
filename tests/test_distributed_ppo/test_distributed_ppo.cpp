#include "bitrl/envs/classic_control/cart_pole.h"
#include "bitrl/envs/parallel_envs.h"
#include "bitrl/maths/statistics/distributions/torch_categorical.h"
#include "bitrl/rl/algorithms/pg/distributed_ppo.h"
#include "bitrl/rl/algorithms/pg/parallel_ppo.h"
#include "bitrl/rl/algorithms/pg/ppo_config.h"
#include "bitrl/rl/distributed/communicator.h"
#include "bitrl/rl/distributed/torch_parameters.h"
#include "bitsim/utils/concurrency/parallel.h"

#include <gtest/gtest.h>
#include <torch/torch.h>

#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace
{
using bitrl::real_t;
using bitrl::torch_tensor_t;
using bitrl::uint_t;
using bitrl::envs::ParallelEnvs;
using bitrl::envs::classic_control::CartPole;
using namespace bitrl::rl::distributed;
namespace pg = bitrl::rl::algos::pg;

// a port of this test run, a different one for every call
uint_t next_port()
{
    static std::atomic<uint_t> n{0};
    return 20000 + static_cast<uint_t>(getpid()) % 20000 + 7 * n++;
}

// run f(communicator) on world ranks, each a thread
void on_ranks(uint_t world, const std::function<void(Communicator &)> &f)
{
    const uint_t port = next_port();
    std::vector<std::thread> threads;
    for (uint_t r = 0; r < world; ++r)
    {
        threads.emplace_back(
            [&, r]
            {
                TCPStoreCommunicator c("127.0.0.1", port, r, world, std::chrono::seconds(20));
                f(c);
            });
    }
    for (auto &t : threads)
    {
        t.join();
    }
}

TEST(TestTCPStoreCommunicator, GathersReducesAndBroadcasts)
{
    const uint_t W = 3;
    std::vector<std::vector<std::string>> gathered(W);
    std::vector<std::vector<double>> sums(W);
    std::vector<std::vector<float>> broadcast(W);
    on_ranks(W,
             [&](Communicator &c)
             {
                 const auto r = c.rank();
                 EXPECT_EQ(c.world_size(), W);
                 gathered[r] = c.all_gather("rank" + std::to_string(r));
                 std::vector<double> v{1.0 * r, 0.1};
                 c.all_reduce_sum(v);
                 sums[r] = v;
                 std::vector<float> b{float(r), float(r)};
                 c.broadcast(b, 1);
                 broadcast[r] = b;
                 c.barrier();
             });
    for (uint_t r = 0; r < W; ++r)
    {
        EXPECT_EQ(gathered[r], (std::vector<std::string>{"rank0", "rank1", "rank2"}));
        EXPECT_EQ(sums[r], sums[0]);
        EXPECT_DOUBLE_EQ(sums[r][0], 3.0);
        EXPECT_EQ(broadcast[r], (std::vector<float>{1.0f, 1.0f}));
    }
}

TEST(TestTCPStoreCommunicator, KeepsUpOverManyCalls)
{
    std::vector<double> last(2);
    on_ranks(2,
             [&](Communicator &c)
             {
                 std::vector<double> v;
                 for (uint_t k = 0; k < 500; ++k)
                 {
                     v = {double(k), double(c.rank())};
                     if (k % 3 == 0)
                     {
                         c.broadcast(v, k % 2);
                     }
                     else
                     {
                         c.all_reduce_sum(v);
                     }
                 }
                 last[c.rank()] = v[0];
             });
    EXPECT_DOUBLE_EQ(last[0], 998.0); // 499 + 499
    EXPECT_DOUBLE_EQ(last[1], 998.0);
}

TEST(TestTCPStoreCommunicator, RejectsBadArguments)
{
    EXPECT_THROW(TCPStoreCommunicator("127.0.0.1", next_port(), 2, 2), std::invalid_argument);
    EXPECT_THROW(TCPStoreCommunicator("127.0.0.1", 0, 0, 1), std::invalid_argument);
}

TEST(TestTorchParameters, FlattenAndSetRoundTrip)
{
    torch::manual_seed(1);
    torch::nn::Linear a(3, 2), b(3, 2);
    const auto values = flatten_parameters(a->parameters());
    EXPECT_EQ(values.size(), 8u);
    set_parameters(b->parameters(), values);
    EXPECT_EQ(flatten_parameters(b->parameters()), values);
    EXPECT_EQ(parameters_checksum(a->parameters()), parameters_checksum(b->parameters()));

    // no gradients yet: zeros, and set_gradients makes them
    EXPECT_EQ(flatten_gradients(b->parameters()), std::vector<float>(8, 0.0f));
    std::vector<float> g(8);
    for (uint_t i = 0; i < 8; ++i)
    {
        g[i] = 0.5f * i;
    }
    set_gradients(b->parameters(), g);
    EXPECT_EQ(flatten_gradients(b->parameters()), g);
    EXPECT_THROW(set_parameters(b->parameters(), std::vector<float>(7)), std::invalid_argument);
}

struct PolicyImpl : torch::nn::Module
{
    PolicyImpl() : fc(register_module("fc", torch::nn::Linear(4, 2))) {}
    torch_tensor_t forward(torch_tensor_t s) { return fc->forward(s); }
    std::tuple<torch_tensor_t, torch_tensor_t> act_batch(torch_tensor_t states)
    {
        bitrl::maths::stats::TorchCategorical d(forward(states), true);
        const auto a = d.sample();
        return {a, d.log_prob(a)};
    }
    std::tuple<torch_tensor_t, torch_tensor_t, torch_tensor_t> evaluate(torch_tensor_t states,
                                                                        torch_tensor_t actions)
    {
        const auto out = forward(states);
        bitrl::maths::stats::TorchCategorical d(out, true);
        return {d.log_prob(actions.view({-1})), d.entropy().mean(), out};
    }
    torch::nn::Linear fc;
};
TORCH_MODULE(Policy);

struct ValueImpl : torch::nn::Module
{
    ValueImpl() : fc(register_module("fc", torch::nn::Linear(4, 1))) {}
    torch_tensor_t forward(torch_tensor_t s) { return fc->forward(s); }
    torch::nn::Linear fc;
};
TORCH_MODULE(Value);

struct Nets
{
    Policy policy;
    Value value;
    std::unique_ptr<torch::optim::Optimizer> po, vo;
    explicit Nets(uint_t seed)
    {
        torch::manual_seed(seed);
        policy = Policy();
        value = Value();
        po = std::make_unique<torch::optim::Adam>(policy->parameters(),
                                                  torch::optim::AdamOptions(1e-2));
        vo = std::make_unique<torch::optim::Adam>(value->parameters(),
                                                  torch::optim::AdamOptions(1e-2));
    }
};

pg::PPOConfig config(uint_t n_rounds)
{
    pg::PPOConfig c;
    c.max_itrs_per_episode = 200;
    c.n_episodes = n_rounds;
    return c;
}

TEST(TestDistributedPPOSolver, TheRanksEndWithTheSameNetworks)
{
    const uint_t W = 3, ROUNDS = 4;
    // every rank starts from networks of its own
    std::vector<std::unique_ptr<Nets>> nets;
    for (uint_t r = 0; r < W; ++r)
    {
        nets.push_back(std::make_unique<Nets>(100 + r));
    }
    std::vector<std::uint64_t> checksums(W);
    std::vector<std::vector<real_t>> returns(W);
    on_ranks(W,
             [&](Communicator &c)
             {
                 const auto r = c.rank();
                 auto &n = *nets[r];
                 pg::DistributedPPOSolver<CartPole, Policy, Value> solver(config(ROUNDS), n.policy,
                                                                          n.value, n.po, n.vo, c);
                 ParallelEnvs<CartPole> envs(2);
                 envs.make("v1", {{"max_steps", uint_t(200)}},
                           {{"seed", bitsim::utils::concurrency::task_seed(7, r)}});
                 envs.use_threads(false);
                 solver.actions_before_training_begins(envs);
                 for (uint_t k = 0; k < ROUNDS; ++k)
                 {
                     solver.on_training_episode(envs, k);
                 }
                 checksums[r] = solver.checksum();
                 returns[r] = solver.global_returns();
             });
    for (uint_t r = 1; r < W; ++r)
    {
        EXPECT_EQ(checksums[r], checksums[0]);
        EXPECT_EQ(returns[r], returns[0]);
    }
    EXPECT_EQ(returns[0].size(), ROUNDS);
    // and they moved from rank 0's start
    Nets start(100);
    EXPECT_NE(checksums[0], parameters_checksum(start.policy->parameters()));
}

TEST(TestDistributedPPOSolver, OneRankIsParallelPPO)
{
    const uint_t ROUNDS = 3;
    auto play = [&](bool distributed)
    {
        Nets n(9);
        ParallelEnvs<CartPole> envs(3);
        envs.make("v1", {{"max_steps", uint_t(200)}}, {});
        torch::manual_seed(11); // the sampling
        std::vector<real_t> all;
        if (distributed)
        {
            TCPStoreCommunicator c("127.0.0.1", next_port(), 0, 1);
            pg::DistributedPPOSolver<CartPole, Policy, Value> solver(config(ROUNDS), n.policy,
                                                                     n.value, n.po, n.vo, c);
            solver.actions_before_training_begins(envs);
            for (uint_t k = 0; k < ROUNDS; ++k)
            {
                solver.on_training_episode(envs, k);
                all.insert(all.end(), solver.last_returns().begin(), solver.last_returns().end());
            }
        }
        else
        {
            pg::ParallelPPOSolver<CartPole, Policy, Value> solver(config(ROUNDS), n.policy, n.value,
                                                                  n.po, n.vo);
            solver.actions_before_training_begins(envs);
            for (uint_t k = 0; k < ROUNDS; ++k)
            {
                solver.on_training_episode(envs, k);
                all.insert(all.end(), solver.last_returns().begin(), solver.last_returns().end());
            }
        }
        return std::make_tuple(all, parameters_checksum(n.policy->parameters()));
    };
    const auto [returns_d, sum_d] = play(true);
    const auto [returns_p, sum_p] = play(false);
    EXPECT_EQ(returns_d, returns_p);
    EXPECT_EQ(sum_d, sum_p);
}

} // namespace
