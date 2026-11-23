#include "bitrl/base/bitrl_config.h"
#include "bitrl/envs/classic_control/cart_pole.h"
#include "bitrl/envs/classic_control/cart_pole_swing_up.h"
#include "bitrl/rl/algorithms/es/evolution_strategy.h"
#include "bitrl/rl/algorithms/es/mlp_policy.h"
#include "bitsim/utils/concurrency/parallel.h"
#include "bitsim/utils/concurrency/thread_pool.h"

#include <gtest/gtest.h>

#include <unistd.h>

#include <atomic>
#include <cmath>
#include <thread>
#include <vector>

namespace
{
using bitrl::real_t;
using bitrl::uint_t;
using namespace bitrl::rl::algos::es;

// minus the squared distance to {1, -2, 3}: the fittest parameters are the target
real_t to_target(const std::vector<real_t> &p, uint_t, uint_t)
{
    const real_t target[3] = {1.0, -2.0, 3.0};
    real_t d = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        d += (p[i] - target[i]) * (p[i] - target[i]);
    }
    return -d;
}

ESConfig small_config()
{
    ESConfig c;
    c.n_pairs = 16;
    c.sigma = 0.1;
    c.learning_rate = 0.05;
    c.weight_decay = 0.0;
    return c;
}

TEST(TestPortableNormal, IsStandardNormalAndRepeatable)
{
    PortableNormal a(7), b(7);
    const int N = 200000;
    real_t sum = 0.0, sum2 = 0.0;
    for (int i = 0; i < N; ++i)
    {
        const real_t x = a();
        EXPECT_EQ(x, b());
        sum += x;
        sum2 += x * x;
    }
    EXPECT_NEAR(sum / N, 0.0, 0.01);
    EXPECT_NEAR(sum2 / N, 1.0, 0.01);
}

TEST(TestEvolutionStrategy, CentredRanks)
{
    const auto r = EvolutionStrategy::centred_ranks({3.0, -1.0, 10.0, 3.0, 0.0});
    EXPECT_EQ(r, (std::vector<real_t>{0.125, -0.5, 0.5, 0.125, -0.25}));
    // all equal: no direction is better
    EXPECT_EQ(EvolutionStrategy::centred_ranks({2.0, 2.0, 2.0}), (std::vector<real_t>(3, 0.0)));
    EXPECT_EQ(EvolutionStrategy::centred_ranks({4.0}), (std::vector<real_t>{0.0}));
}

TEST(TestEvolutionStrategy, RejectsBadSettings)
{
    EXPECT_THROW(EvolutionStrategy(small_config(), {}), std::invalid_argument);
    auto c = small_config();
    c.sigma = 0.0;
    EXPECT_THROW(EvolutionStrategy(c, {1.0}), std::invalid_argument);
}

TEST(TestEvolutionStrategy, ClimbsToTheTarget)
{
    for (const bool shaping : {true, false})
    {
        auto c = small_config();
        c.rank_shaping = shaping;
        EvolutionStrategy es(c, {0.0, 0.0, 0.0});
        for (int g = 0; g < 300; ++g)
        {
            es.step(to_target);
        }
        EXPECT_EQ(es.generation(), 300u);
        EXPECT_GT(to_target(es.parameters(), 0, 0), -0.05) << "rank shaping " << shaping;
    }
}

TEST(TestEvolutionStrategy, PairsShareTheirNoiseAndIndex)
{
    EvolutionStrategy es(small_config(), {0.5, 0.5, 0.5});
    std::vector<std::vector<real_t>> seen(32);
    std::atomic<int> bad{0};
    es.step(
        [&](const std::vector<real_t> &p, uint_t g, uint_t k)
        {
            if (g != 0 || k >= 16)
            {
                ++bad;
            }
            // which member of the pair this is: compare with the noise
            const auto eps = es.noise(0, k);
            const bool plus = std::abs(p[0] - (0.5 + 0.1 * eps[0])) < 1e-12;
            seen[2 * k + (plus ? 0 : 1)] = p;
            return 0.0;
        });
    EXPECT_EQ(bad.load(), 0);
    for (uint_t k = 0; k < 16; ++k)
    {
        for (int j = 0; j < 3; ++j)
        {
            // theta + sigma eps and theta - sigma eps
            EXPECT_NEAR(seen[2 * k][j] + seen[2 * k + 1][j], 1.0, 1e-12);
        }
    }
}

TEST(TestEvolutionStrategy, TheSameOnAnyNumberOfThreads)
{
    auto run = [](bitsim::utils::concurrency::ThreadPool &pool)
    {
        EvolutionStrategy es(small_config(), {0.0, 0.0, 0.0});
        for (int g = 0; g < 20; ++g)
        {
            es.step(to_target, pool);
        }
        return es.checksum();
    };
    bitsim::utils::concurrency::ThreadPool one(1), four(4);
    EXPECT_EQ(run(one), run(four));
}

TEST(TestMLPPolicy, CountsAndComputes)
{
    MLPPolicy net({2, 3, 1});
    EXPECT_EQ(net.n_parameters(), 3u * 3u + 1u * 4u);
    // hidden weights, row by row: unit 0 = (1, 1), unit 1 = (0, 0), unit 2 = (-1, 0); biases 0
    // output: h0 + h1 + h2 + 0.5
    const std::vector<real_t> p{1, 1, 0, 0, -1, 0, 0, 0, 0, 1, 1, 1, 0.5};
    const std::vector<real_t> x{0.3, 0.2};
    const auto out = net.forward(p, x);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_NEAR(out[0], std::tanh(0.5) + 0.0 + std::tanh(-0.3) + 0.5, 1e-12);
    EXPECT_EQ(net.initial_parameters(3), net.initial_parameters(3));
    EXPECT_EQ(net.initial_parameters(3).size(), net.n_parameters());
    EXPECT_THROW(net.forward(std::vector<real_t>(5), x), std::invalid_argument);
}

TEST(TestEvolutionStrategy, LearnsToBalanceThePole)
{
    using bitrl::envs::classic_control::CartPole;
    const MLPPolicy net({4, 8, 2});
    auto fitness = [&net](const std::vector<real_t> &p, uint_t g, uint_t k)
    {
        CartPole env;
        env.make("v1", {{"max_steps", uint_t(200)}}, {});
        return episode_return(
            env, [&](const std::vector<real_t> &obs) { return net.act_discrete(p, obs); },
            bitsim::utils::concurrency::task_seed(g, k), 200);
    };
    ESConfig c;
    c.n_pairs = 16;
    EvolutionStrategy es(c, net.initial_parameters(1));
    real_t mean = 0.0;
    for (int g = 0; g < 40 && mean < 195.0; ++g)
    {
        mean = es.step(fitness).mean_fitness;
    }
    EXPECT_GE(mean, 195.0);
}

TEST(TestCartPoleSwingUp, StartsDownAndPaysForUp)
{
    using bitrl::envs::classic_control::CartPoleSwingUp;
    CartPoleSwingUp env;
    EXPECT_THROW(env.reset(1), std::logic_error);
    env.make("v1", {{"max_steps", uint_t(50)}}, {});
    auto step = env.reset(3);
    const auto obs = step.observation();
    ASSERT_EQ(obs.size(), 5u);
    EXPECT_LT(obs[3], -0.99); // cos(theta): hanging down
    EXPECT_GT(std::abs(env.angle()), 3.0);
    EXPECT_EQ(env.reset(3).observation(), obs);
    EXPECT_THROW(env.step({1.0, 2.0}), std::invalid_argument);

    // doing nothing: the pole hangs, worth about 0 a step, until max_steps
    real_t total = 0.0;
    uint_t n = 0;
    while (!step.done())
    {
        step = env.step({0.0});
        total += step.reward();
        ++n;
        EXPECT_FALSE(step.get_extra<bool>("upright"));
    }
    EXPECT_EQ(n, 50u);
    EXPECT_TRUE(step.get_extra<bool>("truncated"));
    EXPECT_LT(total, 0.1);

    // pushing one way leaves the track, which costs the penalty and ends the episode
    env.reset(3);
    step = env.step({100.0}); // limited to max_force
    while (!step.done())
    {
        step = env.step({100.0});
    }
    EXPECT_FALSE(step.get_extra<bool>("truncated"));
    EXPECT_DOUBLE_EQ(step.reward(), -100.0);
    EXPECT_LT(env.n_steps(), 50u);
}

} // namespace

#ifdef USE_PYTORCH
#include "bitrl/rl/distributed/communicator.h"

namespace
{
TEST(TestEvolutionStrategy, RanksMatchOneProcess)
{
    using bitrl::rl::distributed::Communicator;
    using bitrl::rl::distributed::TCPStoreCommunicator;
    const int G = 15;
    EvolutionStrategy alone(small_config(), {0.0, 0.0, 0.0});
    for (int g = 0; g < G; ++g)
    {
        alone.step(to_target);
    }

    const uint_t W = 3;
    const uint_t port = 20000 + static_cast<uint_t>(getpid()) % 20000 + 3;
    std::vector<std::uint64_t> sums(W);
    std::vector<std::vector<real_t>> fitnesses(W);
    std::vector<std::thread> threads;
    for (uint_t r = 0; r < W; ++r)
    {
        threads.emplace_back(
            [&, r]
            {
                TCPStoreCommunicator c("127.0.0.1", port, r, W, std::chrono::seconds(20));
                EvolutionStrategy es(small_config(), {0.0, 0.0, 0.0});
                bitsim::utils::concurrency::ThreadPool pool(1);
                for (int g = 0; g < G; ++g)
                {
                    es.step(to_target, c, pool);
                }
                sums[r] = es.checksum();
                fitnesses[r] = es.last_fitnesses();
            });
    }
    for (auto &t : threads)
    {
        t.join();
    }
    for (uint_t r = 0; r < W; ++r)
    {
        EXPECT_EQ(sums[r], alone.checksum());
        EXPECT_EQ(fitnesses[r], alone.last_fitnesses());
    }
}
} // namespace
#endif
