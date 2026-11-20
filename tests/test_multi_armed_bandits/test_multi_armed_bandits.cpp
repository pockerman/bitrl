#include "bitrl/base/bitrl_types.h"
#include "bitrl/envs/multi_armed_bandits/multi_armed_bandits.h"

#include <gtest/gtest.h>

#include <any>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
using bitrl::real_t;
using bitrl::uint_t;
using bitrl::envs::bandits::MultiArmedBandits;

void make(MultiArmedBandits &env, const std::vector<real_t> &p)
{
    std::unordered_map<std::string, std::any> options{{"p", std::any(p)}};
    env.make("v0", options, {});
}
} // namespace

TEST(TestMultiArmedBandits, ArmsPayWithTheirProbability)
{
    MultiArmedBandits env;
    make(env, {0.0, 1.0, 0.5});
    ASSERT_EQ(env.n_actions(), 3u);

    real_t fair = 0.0;
    for (uint_t i = 0; i < 1000; ++i)
    {
        EXPECT_DOUBLE_EQ(env.step(0).reward(), 0.0);
        EXPECT_DOUBLE_EQ(env.step(1).reward(), 1.0);
        fair += env.step(2).reward();
    }
    // 500 on average, with a standard deviation of about 16
    EXPECT_NEAR(fair, 500.0, 80.0);
}

// Every pull is an episode of its own; a reset before every pull must not
// make the arm give the same outcome every time
TEST(TestMultiArmedBandits, ResettingDoesNotRepeatThePulls)
{
    MultiArmedBandits env;
    make(env, {0.5});
    uint_t wins = 0;
    for (uint_t i = 0; i < 200; ++i)
    {
        env.reset();
        wins += env.step(0).reward() > 0.0;
    }
    EXPECT_GT(wins, 50u);
    EXPECT_LT(wins, 150u);
}

TEST(TestMultiArmedBandits, TheSameSeedGivesTheSamePulls)
{
    MultiArmedBandits a;
    MultiArmedBandits b;
    make(a, {0.5, 0.3});
    make(b, {0.5, 0.3});
    for (uint_t i = 0; i < 100; ++i)
    {
        EXPECT_EQ(a.step(i % 2).reward(), b.step(i % 2).reward());
    }
}

TEST(TestMultiArmedBandits, AnInvalidArmThrows)
{
    MultiArmedBandits env;
    make(env, {0.5});
    EXPECT_THROW(env.step(1), std::logic_error);
}
