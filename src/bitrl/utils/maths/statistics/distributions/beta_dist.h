#ifndef BETA_DIST_H
#define BETA_DIST_H

#include "rlenvs/rlenvs_consts.h"
#include "rlenvs/rlenvs_types_v2.h"
#include "rlenvs/utils/maths/math_utils.h"

#include <boost/random/beta_distribution.hpp>

#include <cmath>
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
/// \brief The beta distribution
/// is a real-valued distribution which produces values in the range [0, 1].
/// It has two parameters, alpha and beta.
///
template <typename RealType = real_t> class BetaDist
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
    explicit BetaDist(result_type alpha = 1.0, result_type std = 1.0);

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
    /// see https://en.wikipedia.org/wiki/Beta_distribution
    ///
    result_type mean() const { return dist_.alpha() / (dist_.alpha() + dist_.beta()); }

    ///
    /// \brief The variance of the distribution.
    /// see https://en.wikipedia.org/wiki/Beta_distribution
    ///
    result_type variance() const;

    ///
    /// \brief Returns the alpha parameter of the distribution
    ///
    result_type alpha() const { return dist_.alpha(); }

    ///
    /// \brief Returns the beta parameter of the distribution
    ///
    result_type beta() const { return dist_.beta(); }

    ///
    /// \brief Reset the underlying distribution
    ///
    void reset() { dist_.reset(); }

    ///
    /// \brief Reset the underlying distribution
    /// with new alpha beta
    ///
    void reset(result_type a, result_type b);

  private:
    ///
    /// \brief The underlying distribution. Mutable
    /// as the API exposes const methods and the compiler
    /// complains
    ///
    mutable boost::random::beta_distribution<RealType> dist_;
};

template <typename RealType>
BetaDist<RealType>::BetaDist(RealType alpha, RealType beta) : dist_(alpha, beta)
{
}

template <typename RealType> void BetaDist<RealType>::reset(result_type a, result_type b)
{

    dist_ = boost::random::beta_distribution<RealType>(a, b);
}

template <typename RealType> RealType BetaDist<RealType>::variance() const
{
    auto a = dist_.alpha();
    auto b = dist_.beta();
    return a * b / (utils::maths::sqr(a + b) * (a + b + 1));
}

template <typename RealType> RealType BetaDist<RealType>::sample() const
{

    std::random_device rd{};
    std::mt19937 gen{rd()};
    return dist_(gen);
}

template <typename RealType> RealType BetaDist<RealType>::sample(uint_t seed) const
{

    std::mt19937 gen{seed};
    return dist_(gen);
}

template <typename RealType>
std::vector<RealType> BetaDist<RealType>::sample_many(uint_t size) const
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
std::vector<RealType> BetaDist<RealType>::sample_many(uint_t size, uint_t seed) const
{

    std::vector<RealType> samples(size);
    std::mt19937 gen(seed);

    for (uint_t i = 0; i < size; ++i)
    {
        samples[i] = dist_(gen);
    }

    return samples;
}

template <typename RealType> RealType BetaDist<RealType>::pdf(RealType x) const
{

    auto alpha = dist_.alpha();
    auto beta = dist_.beta();
    auto beta_func = std::beta(alpha, beta);
    return std::pow(x, alpha - 1) * std::pow(1.0 - x, beta - 1.0) / beta_func;
}

} // namespace stats
} // namespace maths
} // namespace utils
} // namespace rlenvscpp

#endif