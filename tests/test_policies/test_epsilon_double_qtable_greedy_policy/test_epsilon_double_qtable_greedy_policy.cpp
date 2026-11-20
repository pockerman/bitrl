#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/policies/deterministic_action_policy.h"
#include "bitrl/rl/policies/epsilon_double_qtable_greedy_policy.h"
#include "bitrl/rl/policies/policy_base.h"

#include <gtest/gtest.h>

#include <set>
#include <sstream>
#include <vector>

namespace
{
using bitrl::DynMat;
using bitrl::real_t;
using bitrl::uint_t;
using bitrl::rl::EpsilonDecayOptionType;
using bitrl::rl::policies::EpsilonDoubleQTableGreedyPolicy;

// two states, three actions. In state 0 action 2 has the largest sum of the
// two tables although action 0 is the largest in the first one
DynMat<real_t> q1() { return (DynMat<real_t>(2, 3) << 5.0, 1.0, 3.0, 0.0, 4.0, 1.0).finished(); }
DynMat<real_t> q2() { return (DynMat<real_t>(2, 3) << 0.0, 1.0, 3.0, 0.0, 1.0, 9.0).finished(); }
} // namespace

TEST(TestEpsilonDoubleQTableGreedyPolicy, WithZeroEpsilonPicksTheBestSumOfTheTables)
{
    EpsilonDoubleQTableGreedyPolicy<DynMat<real_t>> policy(0.0, 3, EpsilonDecayOptionType::NONE);
    const auto a = q1();
    const auto b = q2();
    EXPECT_EQ(policy(a, b, uint_t(0)), uint_t(2));
    EXPECT_EQ(policy(a, b, uint_t(1)), uint_t(2));
}

TEST(TestEpsilonDoubleQTableGreedyPolicy, WithEpsilonOnePicksEveryAction)
{
    // a policy that made a new generator in every call would pick the same
    // action every time
    EpsilonDoubleQTableGreedyPolicy<DynMat<real_t>> policy(1.0, 3, EpsilonDecayOptionType::NONE,
                                                           0.01, 1.0, 0.2, 42);
    const auto a = q1();
    const auto b = q2();
    std::set<uint_t> actions;
    for (int i = 0; i < 200; ++i)
    {
        const auto action = policy(a, b, uint_t(0));
        ASSERT_LT(action, uint_t(3));
        actions.insert(action);
    }
    EXPECT_EQ(actions.size(), 3u);
}

TEST(TestEpsilonDoubleQTableGreedyPolicy, TheSameSeedGivesTheSameActions)
{
    EpsilonDoubleQTableGreedyPolicy<DynMat<real_t>> p1(0.5, 3, EpsilonDecayOptionType::NONE, 0.01,
                                                       1.0, 0.2, 7);
    EpsilonDoubleQTableGreedyPolicy<DynMat<real_t>> p2(0.5, 3, EpsilonDecayOptionType::NONE, 0.01,
                                                       1.0, 0.2, 7);
    const auto a = q1();
    const auto b = q2();
    for (int i = 0; i < 50; ++i)
    {
        EXPECT_EQ(p1(a, b, uint_t(1)), p2(a, b, uint_t(1)));
    }
}

TEST(TestEpsilonDoubleQTableGreedyPolicy, ResetRestoresTheInitialEpsilon)
{
    EpsilonDoubleQTableGreedyPolicy<DynMat<real_t>> policy(1.0, 3,
                                                           EpsilonDecayOptionType::INVERSE_STEP);
    policy.adjust_on_episode(4);
    EXPECT_DOUBLE_EQ(policy.eps_value(), 0.25);
    policy.reset();
    EXPECT_DOUBLE_EQ(policy.eps_value(), 1.0);
}

TEST(TestDeterministicActionPolicy, ReturnsTheActionOfTheState)
{
    using bitrl::rl::policies::DeterministicActionPolicy;
    DeterministicActionPolicy<std::vector<uint_t>, uint_t> policy(std::vector<uint_t>{2, 0, 1});
    EXPECT_EQ(policy.on_state(0), uint_t(2));
    EXPECT_EQ(policy.on_state(1), uint_t(0));
    EXPECT_EQ(policy.on_state(2), uint_t(1));
}

namespace
{
class UniformPolicy final : public bitrl::rl::policies::PolicyBase
{
  public:
    UniformPolicy() : PolicyBase(bitrl::rl::policies::PolicyType::DISCRETE_UNIFORM) {}
    std::ostream &print(std::ostream &out) const override { return out << "uniform"; }
};
} // namespace

TEST(TestPolicyBase, KeepsItsTypeAndPrints)
{
    const UniformPolicy policy;
    EXPECT_TRUE(policy.type() == bitrl::rl::policies::PolicyType::DISCRETE_UNIFORM);
    std::ostringstream out;
    out << policy;
    EXPECT_EQ(out.str(), "uniform");
}
