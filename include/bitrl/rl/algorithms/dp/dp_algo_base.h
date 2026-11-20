#ifndef DP_ALGO_BASE_H
#define DP_ALGO_BASE_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/algorithms/rl_algorithm_base.h"

#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

namespace bitrl
{
namespace rl::algos::dp
{

///
/// \brief The DPSolverBase class
///
template <typename EnvType> class DPSolverBase : public RLSolverBase<EnvType>
{
  public:
    ///
    /// \brief The environment type the  solver is using
    ///
    typedef typename RLSolverBase<EnvType>::env_type env_type;

    // state type should be integral
    static_assert(std::is_integral<typename EnvType::state_type>::value);
    static_assert(std::is_integral<typename EnvType::action_type>::value);

    ///
    /// \brief Destructor
    ///
    virtual ~DPSolverBase() = default;

  protected:
    ///
    /// \brief DPAlgoBase
    /// \param name
    ///
    DPSolverBase() = default;
};

} // namespace rl::algos::dp
} // namespace bitrl

#endif // DP_ALGO_BASE_H
