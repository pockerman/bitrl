#ifndef SOFTMAX_POLICY_H
#define SOFTMAX_POLICY_H

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/vector_math.h"
#include "bitrl/rl/policies/max_tabular_policy.h"

namespace bitrl
{
namespace rl
{
namespace policies
{

/**
 * @todo write docs
 */
class MaxTabularSoftmaxPolicy
{
  public:
    /**
     * @brief The output type of operator()
     */
    typedef uint_t output_type;

    /**
     * @brief Constructor
     */
    MaxTabularSoftmaxPolicy(real_t tau = 1.0);

    /**
     * @brief operator(). Given a
     */
    template <typename MatType>
    output_type operator()(const MatType &q_map, uint_t state_idx) const;

    /**
     * @brief operator(). Given a vector always returns the position
     * of the maximum occuring element. If the given vector is empty returns
     * CubeAIConsts::invalid_size_type
     */
    template <typename VecTp> output_type operator()(const VecTp &q_map) const;

    ///
    /// \brief any actions the policy should perform
    /// on the given episode index
    ///
    void on_episode(uint_t) noexcept {}

    ///
    /// \brief Reset the policy
    ///
    void reset() noexcept {}

  private:
    ///
    /// \brief the tau the policy is using
    ///
    real_t tau_;

    MaxTabularPolicy max_policy_;
};

inline MaxTabularSoftmaxPolicy::MaxTabularSoftmaxPolicy(real_t tau) : tau_(tau) {}

template <typename VecTp>
MaxTabularSoftmaxPolicy::output_type MaxTabularSoftmaxPolicy::operator()(const VecTp &q_map) const
{

    auto softmax_vec = maths::softmax_vec(q_map.begin(), q_map.end(), tau_);
    return max_policy_.get_action(softmax_vec);
}

} // namespace policies
} // namespace rl
} // namespace bitrl

#endif // SOFTMAX_POLICY_H
