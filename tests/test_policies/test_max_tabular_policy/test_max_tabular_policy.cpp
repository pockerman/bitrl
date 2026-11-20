#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/policies/max_tabular_policy.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace
{

using bitrl::DynMat;
using bitrl::real_t;
using bitrl::uint_t;
using namespace bitrl::rl::policies;

} // namespace

TEST(TestMaxTabularPolicy, Test_Constructor)
{

    MaxTabularPolicy policy;
    std::vector<real_t> vals{1.0, 2.0, 3.0};
    policy.get_action(vals);
}

TEST(TestMaxTabularPolicy, Test_Operator_1)
{

    MaxTabularPolicy policy;

    std::vector<real_t> vals{1.0, 2.0, 3.0};
    auto max_idx = policy.get_action(vals);
    ASSERT_EQ(max_idx, static_cast<uint_t>(2));
}

TEST(TestMaxTabularPolicy, Test_Operator_2)
{

    MaxTabularPolicy policy;

    std::vector<real_t> vals{3.0, 2.0, 1.0};
    auto max_idx = policy.get_action(vals);
    ASSERT_EQ(max_idx, static_cast<uint_t>(0));
}

TEST(TestMaxTabularPolicy, Test_Operator_3)
{

    MaxTabularPolicy policy;

    std::vector<real_t> vals{1.0, 3.0, 2.0};
    auto max_idx = policy.get_action(vals);
    ASSERT_EQ(max_idx, static_cast<uint_t>(1));
}

TEST(TestMaxTabularPolicy, Test_Operator_4)
{

    MaxTabularPolicy policy;

    DynMat<real_t> vals(3, 3);
    vals(0, 0) = 1.0;
    vals(0, 1) = 2.0;
    vals(0, 2) = 3.0;

    vals(1, 0) = 1.0;
    vals(1, 1) = 2.0;
    vals(1, 2) = 3.0;

    vals(2, 0) = 1.0;
    vals(2, 1) = 2.0;
    vals(2, 2) = 3.0;

    auto max_idx = policy.get_action(vals, 0);
    ASSERT_EQ(max_idx, static_cast<uint_t>(2));
}

TEST(TestMaxTabularPolicy, SaveAndLoad)
{
    // a table of 4 states and 3 actions; the best action of state s is s % 3
    bitrl::DynMat<bitrl::real_t> q = bitrl::DynMat<bitrl::real_t>::Zero(4, 3);
    for (bitrl::uint_t s = 0; s < 4; ++s)
    {
        q(s, s % 3) = 1.0;
    }
    bitrl::rl::policies::MaxTabularPolicy policy;
    bitrl::rl::policies::MaxTabularPolicyBuilder().build_from_state_action_function(q, policy);

    const auto path = std::filesystem::temp_directory_path() / "test_max_tabular_policy.csv";
    policy.save(path.string());

    bitrl::rl::policies::MaxTabularPolicy loaded;
    loaded.load(path.string());
    std::filesystem::remove(path);

    ASSERT_EQ(loaded.n_states(), 4u);
    for (bitrl::uint_t s = 0; s < 4; ++s)
    {
        EXPECT_EQ(loaded.on_state(s), s % 3) << "in state " << s;
    }
}

TEST(TestMaxTabularPolicy, LoadRejectsABadFile)
{
    const auto path = std::filesystem::temp_directory_path() / "test_max_tabular_policy_bad.csv";
    {
        std::ofstream file(path);
        file << "#state,action\n0,1\n2,0\n";
    }
    bitrl::rl::policies::MaxTabularPolicy policy;
    EXPECT_THROW(policy.load(path.string()), std::runtime_error);
    EXPECT_THROW(policy.load("/no/such/file.csv"), std::runtime_error);
    std::filesystem::remove(path);
}
