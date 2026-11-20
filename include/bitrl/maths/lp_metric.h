#ifndef LP_METRIC_H
#define LP_METRIC_H

#include "cubeai/base/cubeai_types.h"
#include <vector>

namespace bitrl
{
namespace maths
{

/// \brief The LpMetric class
/// It conforms to the metric policy

template <int P, bool TTakeRoot = true> class LpMetric
{

  public:
    static const int Power = P;
    static const bool TakeRoot = TTakeRoot;
    static real_t tolerance_value;
    typedef real_t cost_type;
    typedef real_t value_type;

    ///
    /// \brief Tolerance used by the class
    ///
    static real_t tolerance() { return tolerance_value; }

    ///
    /// \brief Default constructor. It is required to
    /// satisfy the metric policy
    ///
    LpMetric() = default;

    ///
    /// \brief Overload operator()
    ///
    real_t operator()(const DynVec<real_t> &v1, const DynVec<real_t> &v2) const;

    ///
    /// \brief evaluate
    ///
    static real_t evaluate(const DynVec<real_t> &v1, const DynVec<real_t> &v2);

    ///
    /// \brief evaluate
    ///
    static real_t evaluate(const std::vector<real_t> &v1, const std::vector<real_t> &v2);
};

/// \brief some useful shortcuts
using ManhattanMetric = LpMetric<1, false>;
using SqrEuclidean_metric = LpMetric<2, false>;
using EuclideanMetric = LpMetric<2, true>;

} // namespace maths
} // namespace bitrl

#include "cubeai/maths/lp_metric_impl.h"

#endif // LP_METRIC_H
