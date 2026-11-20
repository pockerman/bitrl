#ifndef LP_METRIC_IMPL_H
#define LP_METRIC_IMPL_H

#include <cmath>
#include <stdexcept>
#include <string>

namespace bitrl
{
namespace maths
{

template <int P, bool TTakeRoot> real_t LpMetric<P, TTakeRoot>::tolerance_value = 1.0e-8;

template <int P, bool TTakeRoot>
real_t LpMetric<P, TTakeRoot>::evaluate(const DynVec<real_t> &v1, const DynVec<real_t> &v2)
{
    static_assert(P >= 1, "LpMetric needs P >= 1");

    if (v1.size() != v2.size())
    {
        throw std::invalid_argument("LpMetric: the vectors have sizes " +
                                    std::to_string(v1.size()) + " and " +
                                    std::to_string(v2.size()));
    }

    const auto diff = (v1 - v2).cwiseAbs();
    if constexpr (P == 1)
    {
        // the root of the L1 norm is the norm itself
        return diff.sum();
    }
    else if constexpr (P == 2)
    {
        const auto sqr_norm = diff.squaredNorm();
        return TTakeRoot ? std::sqrt(sqr_norm) : sqr_norm;
    }
    else
    {
        const real_t sum = diff.array().pow(static_cast<real_t>(P)).sum();
        return TTakeRoot ? std::pow(sum, 1.0 / P) : sum;
    }
}

template <int P, bool TTakeRoot>
real_t LpMetric<P, TTakeRoot>::evaluate(const std::vector<real_t> &v1,
                                        const std::vector<real_t> &v2)
{
    using map_type = Eigen::Map<const DynVec<real_t>>;
    return evaluate(DynVec<real_t>(map_type(v1.data(), static_cast<Eigen::Index>(v1.size()))),
                    DynVec<real_t>(map_type(v2.data(), static_cast<Eigen::Index>(v2.size()))));
}

template <int P, bool TTakeRoot>
real_t LpMetric<P, TTakeRoot>::operator()(const DynVec<real_t> &v1, const DynVec<real_t> &v2) const
{
    return LpMetric<P, TTakeRoot>::evaluate(v1, v2);
}

} // namespace maths
} // namespace bitrl

#endif // LP_METRIC_IMPL_H
