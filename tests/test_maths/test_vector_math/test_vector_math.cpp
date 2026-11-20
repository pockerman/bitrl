#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/vector_math.h"

#include <gtest/gtest.h>
#include <iostream>
#include <vector>

namespace
{

using bitrl::DynVec;
using bitrl::real_t;
using bitrl::uint_t;
} // namespace

TEST(TestSoftMaxVector, Test_1)
{

    DynVec<real_t> vals(4);
    for (uint_t i = 0; i < static_cast<uint_t>(vals.size()); ++i)
    {
        vals[i] = 10.0;
    }

    auto softmax_vec = bitrl::maths::softmax_vec(vals);

    EXPECT_DOUBLE_EQ(0.25, softmax_vec[0]);
    EXPECT_DOUBLE_EQ(0.25, softmax_vec[1]);
    EXPECT_DOUBLE_EQ(0.25, softmax_vec[2]);
    EXPECT_DOUBLE_EQ(0.25, softmax_vec[3]);
}

TEST(TestZeroCenter, Test_2)
{

    DynVec<real_t> vals(4);
    for (uint_t i = 0; i < static_cast<uint_t>(vals.size()); ++i)
    {
        vals[i] = 10.0;
    }

    auto mean = bitrl::maths::mean(vals);
    auto zero_center_mean = bitrl::maths::zero_center(vals);

    EXPECT_DOUBLE_EQ(vals[0] - mean, zero_center_mean[0]);
    EXPECT_DOUBLE_EQ(vals[1] - mean, zero_center_mean[1]);
    EXPECT_DOUBLE_EQ(vals[2] - mean, zero_center_mean[2]);
    EXPECT_DOUBLE_EQ(vals[3] - mean, zero_center_mean[3]);
}

TEST(TestLogSpace, Test_3)
{

    // this is taken from NumPy:
    // https://numpy.org/doc/stable/reference/generated/numpy.logspace.html
    auto vals = bitrl::maths::logspace(2.0, 3.0, 4, 10.0);

    ASSERT_EQ(vals.size(), 4);
    EXPECT_NEAR(vals[0], 100.0, 1.0e-2);
    EXPECT_NEAR(vals[1], 215.44, 1.0e-2);
    EXPECT_NEAR(vals[2], 464.15, 1.0e-2);
    EXPECT_NEAR(vals[3], 1000.0, 1.0e-2);
}

/*
TEST(TestMaxTabularPolicy, Test_Operator_3) {

    RandomTabularPolicy policy(42);

    DynMat<real_t> vals(3, 3);
    vals(0,0) = 1.0;
    vals(0,1) = 2.0;
    vals(0,2) = 3.0;

    vals(1,0) = 1.0;
    vals(1,1) = 2.0;
    vals(1,2) = 3.0;

    vals(2,0) = 1.0;
    vals(2,1) = 2.0;
    vals(2,2) = 3.0;

    auto max_idx = policy(vals, 0);
    ASSERT_EQ(max_idx, max_idx);

}*/
