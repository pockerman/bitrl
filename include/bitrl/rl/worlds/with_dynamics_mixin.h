#ifndef WITH_DYNAMICS_MIXIN_H
#define WITH_DYNAMICS_MIXIN_H

#include "bitrl/base/bitrl_types.h"

#include <tuple>
#include <vector>

namespace bitrl
{
namespace rl
{
namespace envs
{

struct with_dynamics_mixin
{
    static constexpr bool has_dynamics{true};

    ///
    ///
    ///
    virtual ~with_dynamics_mixin() = default;

    ///
    /// \brief transition_dynamics
    ///
    virtual std::vector<std::tuple<real_t, uint_t, real_t, bool>>
    transition_dynamics(uint_t s, uint_t aidx) const = 0;

  protected:
    ///
    ///
    ///
    with_dynamics_mixin() = default;
};

} // namespace envs
} // namespace rl
} // namespace bitrl

#endif // WITH_DYNAMICS_MIXIN_H
