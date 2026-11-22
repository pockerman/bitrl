#include "bitrl/envs/classic_control/cart_pole.h"
#include "bitrl/envs/parallel_envs.h"
#include "bitrl/maths/statistics/distributions/torch_categorical.h"
#include "bitrl/rl/algorithms/pg/parallel_ppo.h"
#include "bitrl/rl/algorithms/pg/ppo_config.h"

#include <gtest/gtest.h>
#include <torch/torch.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace
{
using bitrl::real_t;
using bitrl::torch_tensor_t;
using bitrl::uint_t;
using bitrl::envs::ParallelEnvs;
using bitrl::envs::classic_control::CartPole;
namespace pg = bitrl::rl::algos::pg;

CartPole make_cart_pole(uint_t max_steps = 500, uint_t seed = 42)
{
    CartPole env;
    env.make("v1", {{"max_steps", max_steps}}, {{"seed", seed}});
    return env;
}

// push towards where the pole leans: keeps it up for a long time
uint_t balance(const std::vector<real_t> &o) { return o[2] + 0.5 * o[3] > 0.0 ? 1 : 0; }

TEST(TestCartPole, StartsNearUprightFromTheSeed)
{
    auto env = make_cart_pole();
    const auto a = env.reset(7).observation();
    const auto b = env.reset(7).observation();
    const auto c = env.reset(8).observation();
    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
    for (const real_t v : a)
    {
        EXPECT_LE(std::abs(v), 0.05);
    }
    EXPECT_TRUE(env.reset(7).first());
}

TEST(TestCartPole, FallsWhenPushedOneWay)
{
    auto env = make_cart_pole();
    auto step = env.reset();
    uint_t n = 0;
    while (!step.last())
    {
        step = env.step(1);
        EXPECT_DOUBLE_EQ(step.reward(), 1.0);
        ++n;
    }
    EXPECT_LT(n, 100u);
    EXPECT_FALSE(step.get_extra<bool>("truncated"));
}

TEST(TestCartPole, TruncatesAtTheStepLimit)
{
    auto env = make_cart_pole(200);
    auto step = env.reset();
    uint_t n = 0;
    while (!step.last())
    {
        step = env.step(balance(step.observation()));
        ++n;
    }
    EXPECT_EQ(n, 200u);
    EXPECT_TRUE(step.get_extra<bool>("truncated"));
}

TEST(TestCartPole, RejectsBadUse)
{
    CartPole env;
    EXPECT_THROW(env.reset(), std::logic_error);
    env.make("v1", {}, {});
    env.reset();
    EXPECT_THROW(env.step(2), std::invalid_argument);
    EXPECT_THROW(CartPole().make("v1", {{"dt", 0.0}}, {}), std::invalid_argument);
}

TEST(TestParallelEnvs, SeedsEveryCopyDifferently)
{
    ParallelEnvs<CartPole> envs(4);
    envs.make("v1", {}, {{"seed", uint_t(3)}});
    const auto &first = envs.reset();
    ASSERT_EQ(first.size(), 4u);
    for (uint_t i = 1; i < 4; ++i)
    {
        EXPECT_NE(first[i].observation(), first[0].observation());
    }
}

TEST(TestParallelEnvs, ThreadsChangeNothing)
{
    auto play = [](bool threads)
    {
        ParallelEnvs<CartPole> envs(6);
        envs.make("v1", {{"max_steps", uint_t(300)}}, {});
        envs.use_threads(threads);
        envs.reset();
        std::vector<std::vector<real_t>> seen;
        while (!envs.all_done())
        {
            std::vector<uint_t> actions;
            for (const auto &t : envs.time_steps())
            {
                actions.push_back(balance(t.observation()));
            }
            for (const auto &t : envs.step(actions))
            {
                seen.push_back(t.observation());
            }
        }
        return seen;
    };
    EXPECT_EQ(play(true), play(false));
}

TEST(TestParallelEnvs, ACopyThatIsDoneWaits)
{
    ParallelEnvs<CartPole> envs(2);
    envs.make("v1", {}, {});
    envs.reset();
    // copy 0 falls, copy 1 balances
    while (!envs.done(0))
    {
        envs.step({1, balance(envs.time_steps()[1].observation())});
    }
    const auto steps0 = envs.env(0).n_steps();
    const auto steps1 = envs.env(1).n_steps();
    envs.step({1, balance(envs.time_steps()[1].observation())});
    EXPECT_EQ(envs.env(0).n_steps(), steps0);
    EXPECT_EQ(envs.env(1).n_steps(), steps1 + 1);
    EXPECT_FALSE(envs.all_done());
    EXPECT_THROW(envs.step({1}), std::invalid_argument);
}

TEST(TestParallelEnvs, RejectsBadUse)
{
    EXPECT_THROW(ParallelEnvs<CartPole>(0), std::invalid_argument);
    ParallelEnvs<CartPole> envs(2);
    EXPECT_THROW(envs.reset(), std::logic_error);
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

// a few rounds of ParallelPPOSolver: the returns of every episode and the weights after
std::tuple<std::vector<real_t>, torch_tensor_t> train(bool threads, uint_t n_rounds)
{
    torch::manual_seed(5);
    Policy policy;
    Value value;
    std::unique_ptr<torch::optim::Optimizer> po =
        std::make_unique<torch::optim::Adam>(policy->parameters(), torch::optim::AdamOptions(1e-2));
    std::unique_ptr<torch::optim::Optimizer> vo =
        std::make_unique<torch::optim::Adam>(value->parameters(), torch::optim::AdamOptions(1e-2));
    pg::PPOConfig config;
    config.max_itrs_per_episode = 200;
    config.n_episodes = n_rounds;
    pg::ParallelPPOSolver<CartPole, Policy, Value> solver(config, policy, value, po, vo);

    ParallelEnvs<CartPole> envs(4);
    envs.make("v1", {{"max_steps", uint_t(200)}}, {});
    envs.use_threads(threads);
    solver.actions_before_training_begins(envs);
    std::vector<real_t> returns;
    for (uint_t r = 0; r < n_rounds; ++r)
    {
        const auto info = solver.on_training_episode(envs, r);
        EXPECT_EQ(solver.last_returns().size(), 4u);
        real_t sum = 0.0;
        uint_t steps = 0;
        for (const real_t g : solver.last_returns())
        {
            sum += g;
            steps += static_cast<uint_t>(g); // CartPole gives 1 a step
        }
        EXPECT_DOUBLE_EQ(info.episode_reward, sum / 4.0);
        EXPECT_EQ(info.episode_iterations, steps);
        returns.insert(returns.end(), solver.last_returns().begin(), solver.last_returns().end());
    }
    return {returns, policy->fc->weight.detach().clone()};
}

TEST(TestParallelPPOSolver, LearnsTheSameOnThreads)
{
    const auto [returns_a, weights_a] = train(true, 5);
    const auto [returns_b, weights_b] = train(false, 5);
    EXPECT_EQ(returns_a, returns_b);
    EXPECT_TRUE(torch::equal(weights_a, weights_b));
    EXPECT_EQ(returns_a.size(), 20u);
}

TEST(TestParallelPPOSolver, ChangesThePolicy)
{
    torch::manual_seed(5);
    Policy fresh;
    const auto [returns, weights] = train(false, 3);
    EXPECT_FALSE(torch::equal(weights, fresh->fc->weight));
}

} // namespace
