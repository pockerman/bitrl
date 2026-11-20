#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/policies/max_tabular_policy.h"

#include <gtest/gtest.h>
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
