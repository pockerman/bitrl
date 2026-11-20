#ifndef EPSILON_GREEDY_POLICY_H
#define EPSILON_GREEDY_POLICY_H

#include "bitrl/base/bitrl_config.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/policies/max_tabular_policy.h"
#include "bitrl/rl/policies/random_tabular_policy.h"

#ifdef USE_PYTORCH
#include "bitrl/utils/torch_adaptor.h"
#endif

#include <cmath>
#include <random>

namespace bitrl
{
namespace rl
{
namespace policies
{

///
/// \brief The EpsilonDecayOption enum. Enumerate various decaying options.
///
enum class EpsilonDecayOption
{
    NONE,
    EXPONENTIAL,
    INVERSE_STEP,
    CONSTANT_RATE
};

///
/// \brief The EpsilonGreedyPolicy class
///
class EpsilonGreedyPolicy
{
  public:
    ///
    /// \brief The type returned when calling this->operator()
    ///
    typedef uint_t output_type;

    constexpr static real_t MIN_EPS = 0.01;
    constexpr static real_t MAX_EPS = 1.0;
    constexpr static real_t EPSILON_DECAY_FACTOR = 0.01;

    ///
    /// \brief Constructor. Creates an epsilon-greedy tabular policy
    ///
    EpsilonGreedyPolicy(real_t eps);

    ////
    ///	\brief Constructor. Creates an epsilon-greedy tabular policy
    ///
    explicit EpsilonGreedyPolicy(real_t eps, uint_t seed);

    ///
    /// \brief Constructor Creates an epsilon greedy policy with an
    /// epsilon decay strategy
    ///
    explicit EpsilonGreedyPolicy(real_t eps, uint_t seed, EpsilonDecayOption decay_op,
                                 real_t min_eps = MIN_EPS, real_t max_eps = MAX_EPS,
                                 real_t epsilon_decay = EPSILON_DECAY_FACTOR);

    ///
    /// \brief operator() Select action for the given state
    ///
    template <typename MapType> output_type operator()(const MapType &q_map, uint_t state) const;

    ///
    /// \brief Get an action i.e. index from the given values
    ///
    template <typename VecType> output_type operator()(const VecType &vec) const;

    ///
    /// \brief get_action. Given a
    ///
    template <typename MatType> output_type get_action(const MatType &q_map, uint_t state_idx);

    ///
    /// \brief get_action. Given a vector always returns the position
    /// of the maximum occuring element. If the given vector is empty returns
    /// CubeAIConsts::invalid_size_type
    ///
    template <typename VecTp> output_type get_action(const VecTp &q_map);

#ifdef USE_PYTORCH
    output_type operator()(const torch_tensor_t &vec, torch_tensor_value_type<real_t>) const;
    output_type operator()(const torch_tensor_t &vec, torch_tensor_value_type<float_t>) const;
    output_type operator()(const torch_tensor_t &vec, torch_tensor_value_type<int_t>) const;
    output_type operator()(const torch_tensor_t &vec, torch_tensor_value_type<lint_t>) const;
#endif

    ///
    /// \brief any actions the policy should perform
    /// on the given episode index
    ///
    void on_episode(uint_t episode_idx) noexcept;

    ///
    /// \brief Reset the policy
    ///
    void reset() noexcept { eps_ = eps_init_; }

    ///
    /// \brief Returns the value of the epsilon
    ///
    real_t eps_value() const noexcept { return eps_; }

    ///
    /// \brief Set the epsilon value
    /// \param eps
    ///
    void set_eps_value(real_t eps);

    ///
    /// \brief Returns the decay option
    ///
    EpsilonDecayOption decay_option() const noexcept { return decay_op_; }

  private:
    real_t eps_init_;
    real_t eps_;
    real_t min_eps_;
    real_t max_eps_;
    real_t epsilon_decay_;
    EpsilonDecayOption decay_op_;

    ///
    /// \brief The random engine generator
    ///
    mutable std::mt19937 generator_;

    // how to select the action
    RandomTabularPolicy random_policy_;
    MaxTabularPolicy max_policy_;
};

inline EpsilonGreedyPolicy::EpsilonGreedyPolicy(real_t eps, uint_t seed,
                                                EpsilonDecayOption decay_op, real_t min_eps,
                                                real_t max_eps, real_t epsilon_decay)
    : eps_init_(eps), eps_(eps), min_eps_(min_eps), max_eps_(max_eps),
      epsilon_decay_(epsilon_decay), decay_op_(decay_op), generator_(seed), random_policy_(seed),
      max_policy_()
{
}

inline EpsilonGreedyPolicy::EpsilonGreedyPolicy(real_t eps)
    : eps_init_(eps), eps_(eps), min_eps_(eps), max_eps_(eps), epsilon_decay_(eps),
      decay_op_(EpsilonDecayOption::NONE), random_policy_(), max_policy_()
{
}

inline EpsilonGreedyPolicy::EpsilonGreedyPolicy(real_t eps, uint_t seed)
    : EpsilonGreedyPolicy(eps, seed, EpsilonDecayOption::NONE, eps, eps, eps)
{
}

template <typename VecType>
EpsilonGreedyPolicy::output_type EpsilonGreedyPolicy::operator()(const VecType &vec) const
{

    // generate a number in [0, 1]
    std::uniform_real_distribution<> real_dist_(0.0, 1.0);

    if (real_dist_(generator_) > eps_)
    {
        // select greedy action with probability 1 - epsilon
        return max_policy_.get_action(vec);
    }

    // else select a random action
    return random_policy_(vec);
}

template <typename VecTp>
EpsilonGreedyPolicy::output_type EpsilonGreedyPolicy::get_action(const VecTp &vec)
{
    // generate a number in [0, 1]
    std::uniform_real_distribution<> real_dist_(0.0, 1.0);

    if (real_dist_(generator_) > eps_)
    {
        // select greedy action with probability 1 - epsilon
        return max_policy_.get_action(vec);
    }

    // else select a random action
    return random_policy_(vec);
}

} // namespace policies
} // namespace rl
} // namespace bitrl

#endif // EPSILON_GREEDY_POLICY_H
