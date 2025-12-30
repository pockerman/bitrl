#ifndef UNIFORM_DIST_H
#define UNIFORM_DIST_H

#include "rlenvs/rlenvs_consts.h"
#include "rlenvs/rlenvs_types_v2.h"

#include <initializer_list>
#include <random>
#include <utility>
#include <vector>

namespace rlenvscpp
{
namespace utils
{
namespace maths
{
namespace stats
{

namespace
{
template <typename t> struct uniform_distribution_selector;

template <> struct uniform_distribution_selector<uint_t>
{
    typedef std::uniform_int_distribution<uint_t> distribution_type;
};

template <> struct uniform_distribution_selector<int_t>
{
    typedef std::uniform_int_distribution<int_t> distribution_type;
};

template <> struct uniform_distribution_selector<lint_t>
{
    typedef std::uniform_int_distribution<lint_t> distribution_type;
};

template <> struct uniform_distribution_selector<real_t>
{
    typedef std::uniform_real_distribution<real_t> distribution_type;
};

template <> struct uniform_distribution_selector<float_t>
{
    typedef std::uniform_real_distribution<float_t> distribution_type;
};

} // namespace

///
/// Produces random integer values i, uniformly distributed on the closed interval
/// [a,b], that is, distributed according to the discrete probability function
///
template <typename OutType> class UniformDist
{

    typedef OutType out_type;
    typedef std::pair<out_type, out_type> range_type;

    ///
    /// \brief Construct with [a, b]
    ///
    UniformDist(out_type a, out_type b);

    ///
    /// \brief Construct with [a, b]
    ///
    UniformDist(range_type range);

    ///
    /// \brief Get a sample
    ///
    out_type sample();

    ///
    /// \brief Get a sample
    ///
    out_type sample(uint_t seed);

    ///
    /// returns the [a, b] range
    ///
    range_type range() const { return range_; }

  private:
    range_type range_;
    uniform_distribution_selector<out_type>::distribution_type distribution_;
};

template <typename OutType>
UniformDist<OutType>::UniformDist(typename UniformDist<OutType>::out_type a,
                                  typename UniformDist<OutType>::out_type b)
    : range_(a, b), distribution_(a, b)
{
}

template <typename OutType>
UniformDist<OutType>::UniformDist(typename UniformDist<OutType>::range_type range)
    : UniformDist<OutType>(range.first, range.second)
{
}

template <typename OutType> typename UniformDist<OutType>::out_type UniformDist<OutType>::sample()
{

    // a seed source for the random number engine
    std::random_device rd;

    // mersenne_twister_engine seeded with rd()
    std::mt19937 gen(rd());

    return distribution_(gen);
}

template <typename OutType>
typename UniformDist<OutType>::out_type UniformDist<OutType>::sample(uint_t seed)
{
    // mersenne_twister_engine seeded with seed
    std::mt19937 gen(seed);
    return distribution_(gen);
}

template <typename OutType, typename WeightType = real_t> class UniformWeightedDist
{
  public:
    typedef OutType out_type;
    typedef std::vector<WeightType> weights_type;

    ///
    /// \brief Initialize with a list of weights
    /// see also: https://www.cplusplus.com/reference/random/discrete_distribution/
    ///
    template <typename VectorType> UniformWeightedDist(const VectorType &weights);

    ///
    /// \brief Initialize with a list of weights
    /// see also: https://www.cplusplus.com/reference/random/discrete_distribution/
    ///
    UniformWeightedDist(std::initializer_list<WeightType> weights);

    ///
    /// \brief Get a sample
    ///
    out_type sample();

    ///
    /// \brief Get a sample
    ///
    out_type sample(uint_t seed);

    ///
    /// returns the [a, b] range
    ///
    std::vector<real_t> probabilities() const { return distribution_.probabilities(); }

  private:
    weights_type weights_;
    std::discrete_distribution<out_type> distribution_;
};

template <typename OutType, typename WeightType>
template <typename VectorType>
UniformWeightedDist<OutType, WeightType>::UniformWeightedDist(const VectorType &weights)
    : weights_(weights), distribution_(weights.begin(), weights.end())
{
}

template <typename OutType, typename WeightType>
UniformWeightedDist<OutType, WeightType>::UniformWeightedDist(
    std::initializer_list<WeightType> weights)
    : weights_(weights), distribution_(weights)
{
}

template <typename OutType, typename WeightType>
typename UniformWeightedDist<OutType, WeightType>::out_type
UniformWeightedDist<OutType, WeightType>::sample()
{

    // a seed source for the random number engine
    std::random_device rd;

    // mersenne_twister_engine seeded with rd()
    std::mt19937 gen(rd());
    return distribution_(gen);
}

template <typename OutType, typename WeightType>
typename UniformWeightedDist<OutType, WeightType>::out_type
UniformWeightedDist<OutType, WeightType>::sample(uint_t seed)
{
    // mersenne_twister_engine seeded with seed
    std::mt19937 gen(seed);
    return distribution_(gen);
}

} // namespace stats
} // namespace maths
} // namespace utils
} // namespace rlenvscpp

#endif