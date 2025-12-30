#ifndef NORMAL_DIST_H
#define NORMAL_DIST_H

#include "rlenvs/rlenvs_consts.h"
#include "rlenvs/rlenvs_types_v2.h"

#include <cmath>
#include <numbers>
#include <random>
#include <type_traits>
#include <vector>

namespace rlenvscpp
{
namespace utils
{
namespace maths
{
namespace stats
{

///
/// \brief Wrapper to std::normal_distribution to facilitate
/// sampling multiple values, sampling with a given seed and
/// computing the PDF value at a specific point
///
template <typename RealType = real_t> class NormalDist
{

  public:
    static_assert(std::is_floating_point<RealType>::value, "Not a floating point type");

    ///
    /// \breif The return type every time we call pdf, sample
    ///
    typedef RealType result_type;

    ///
    /// \brief Constructor
    ///
    NormalDist();

    ///
    /// \brief Constructor
    ///
    explicit NormalDist(result_type mu, result_type std = 1.0);

    ///
    /// \brief compute the value of the PDF at the given point
    ///
    result_type pdf(result_type x) const;

    ///
    /// \brief Sample from the distribution
    ///
    result_type sample() const;

    ///
    /// \brief Sample from the distribution
    ///
    result_type sample(uint_t seed) const;

    ///
    /// \brief sample from the distribution
    ///
    std::vector<result_type> sample_many(uint_t size) const;

    ///
    /// \brief sample from the distribution
    ///
    std::vector<result_type> sample_many(uint_t size, uint_t seed) const;

    ///
    /// \brief The mean value of the distribution
    ///
    result_type mean() const { return dist_.mean(); }

    ///
    /// \brief The STD of the distribution
    ///
    result_type std() const { return dist_.stddev(); }

  private:
    ///
    /// \brief The underlying distribution. Mutable
    /// as the API exposes const methods and the compiler
    /// complains
    ///
    mutable std::normal_distribution<RealType> dist_;
};

template <typename RealType>
NormalDist<RealType>::NormalDist(RealType mu, RealType std) : dist_(mu, std)
{
}

template <typename RealType> NormalDist<RealType>::NormalDist() : NormalDist<RealType>(0.0, 1.0) {}

template <typename RealType> RealType NormalDist<RealType>::sample() const
{

    std::random_device rd{};
    std::mt19937 gen{rd()};
    return dist_(gen);
}

template <typename RealType> RealType NormalDist<RealType>::sample(uint_t seed) const
{

    std::mt19937 gen{seed};
    return dist_(gen);
}

template <typename RealType>
std::vector<RealType> NormalDist<RealType>::sample_many(uint_t size) const
{

    std::vector<RealType> samples(size);
    std::random_device rd{};
    std::mt19937 gen{rd()};

    for (uint_t i = 0; i < size; ++i)
    {
        samples[i] = dist_(gen);
    }

    return samples;
}

template <typename RealType>
std::vector<RealType> NormalDist<RealType>::sample_many(uint_t size, uint_t seed) const
{

    std::vector<RealType> samples(size);
    std::mt19937 gen(seed);

    for (uint_t i = 0; i < size; ++i)
    {
        samples[i] = dist_(gen);
    }

    return samples;
}

template <typename RealType> RealType NormalDist<RealType>::pdf(RealType x) const
{

    auto mu = dist_.mean();
    auto std = dist_.stddev();
    auto pi = rlenvscpp::consts::maths::PI;
    auto factor = 1.0 / (std * std::sqrt(2.0 * pi));
    auto exp = std::exp(-0.5 * std::pow((x - mu) / std, 2.0));
    return factor * exp;
}

} // namespace stats
} // namespace maths
} // namespace utils
} // namespace rlenvscpp

#endif
