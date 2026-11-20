#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/lp_metric.h"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace
{
using bitrl::DynVec;
using bitrl::real_t;
using bitrl::maths::EuclideanMetric;
using bitrl::maths::LpMetric;
using bitrl::maths::ManhattanMetric;
using bitrl::maths::SqrEuclidean_metric;

// the difference of the two vectors is (3, -4, 0)
DynVec<real_t> v1() { return (DynVec<real_t>(3) << 1.0, 2.0, 5.0).finished(); }
DynVec<real_t> v2() { return (DynVec<real_t>(3) << -2.0, 6.0, 5.0).finished(); }
} // namespace

TEST(TestLpMetric, L1)
{
    EXPECT_DOUBLE_EQ(ManhattanMetric::evaluate(v1(), v2()), 7.0);
    EXPECT_DOUBLE_EQ((LpMetric<1, true>::evaluate(v1(), v2())), 7.0);
}

TEST(TestLpMetric, L2)
{
    EXPECT_DOUBLE_EQ(EuclideanMetric::evaluate(v1(), v2()), 5.0);
    EXPECT_DOUBLE_EQ(SqrEuclidean_metric::evaluate(v1(), v2()), 25.0);
    EXPECT_DOUBLE_EQ(EuclideanMetric()(v1(), v2()), 5.0);
}

TEST(TestLpMetric, L3)
{
    // 3^3 + 4^3 = 91
    EXPECT_DOUBLE_EQ((LpMetric<3, false>::evaluate(v1(), v2())), 91.0);
    EXPECT_NEAR((LpMetric<3, true>::evaluate(v1(), v2())), std::cbrt(91.0), 1.0e-12);
}

TEST(TestLpMetric, StdVectorGivesTheSameValues)
{
    const std::vector<real_t> a = {1.0, 2.0, 5.0};
    const std::vector<real_t> b = {-2.0, 6.0, 5.0};
    EXPECT_DOUBLE_EQ(ManhattanMetric::evaluate(a, b), 7.0);
    EXPECT_DOUBLE_EQ(EuclideanMetric::evaluate(a, b), 5.0);
    EXPECT_DOUBLE_EQ(SqrEuclidean_metric::evaluate(a, b), 25.0);
    EXPECT_DOUBLE_EQ((LpMetric<3, false>::evaluate(a, b)), 91.0);
}

TEST(TestLpMetric, TheSameVectorIsAtDistanceZero)
{
    EXPECT_DOUBLE_EQ(EuclideanMetric::evaluate(v1(), v1()), 0.0);
}

TEST(TestLpMetric, VectorsOfDifferentSizesThrow)
{
    const DynVec<real_t> shorter = DynVec<real_t>::Zero(2);
    EXPECT_THROW(EuclideanMetric::evaluate(v1(), shorter), std::invalid_argument);
}
