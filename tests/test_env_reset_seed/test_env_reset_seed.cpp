// Every reset after make() must use the next seed: the same seed every time
// would start every episode from the same state.
#include "bitrl/envs/env_base.h"
#include "bitrl/envs/env_types.h"
#include "bitrl/envs/time_step.h"

#include <gtest/gtest.h>

#include <any>
#include <string>
#include <unordered_map>

namespace
{
using bitrl::TimeStep;
using bitrl::uint_t;

// an environment that only records the seed of every reset
class SeedEnv final
    : public bitrl::envs::EnvBase<TimeStep<uint_t>, bitrl::envs::ScalarDiscreteEnv<2, 2, 0, 0>>
{
  public:
    using base_type =
        bitrl::envs::EnvBase<TimeStep<uint_t>, bitrl::envs::ScalarDiscreteEnv<2, 2, 0, 0>>;

    SeedEnv() : base_type("0", "SeedEnv") {}

    void make(const std::string &version, const std::unordered_map<std::string, std::any> &options,
              const std::unordered_map<std::string, std::any> &reset_options) override
    {
        base_type::make(version, options, reset_options);
        this->make_created_();
    }
    void close() override {}
    time_step_type reset() override
    {
        seed = this->next_reset_seed_();
        return time_step_type();
    }
    time_step_type step(const action_type &) override { return time_step_type(); }

    uint_t seed{0};
};
} // namespace

TEST(TestEnvResetSeed, DefaultSeedThenNext)
{
    SeedEnv env;
    env.make("v0", {}, {});
    env.reset();
    EXPECT_EQ(env.seed, SeedEnv::DEFAULT_ENV_SEED);
    env.reset();
    EXPECT_EQ(env.seed, SeedEnv::DEFAULT_ENV_SEED + 1);
    env.reset();
    EXPECT_EQ(env.last_reset_seed(), SeedEnv::DEFAULT_ENV_SEED + 2);
}

TEST(TestEnvResetSeed, GivenSeedThenNextAndMakeStartsAgain)
{
    SeedEnv env;
    std::unordered_map<std::string, std::any> reset_options{{"seed", std::any(uint_t(7))}};
    env.make("v0", {}, reset_options);
    env.reset();
    env.reset();
    EXPECT_EQ(env.last_reset_seed(), 8u);

    // a new make() starts the seeds again
    env.make("v0", {}, reset_options);
    env.reset();
    EXPECT_EQ(env.last_reset_seed(), 7u);
}
