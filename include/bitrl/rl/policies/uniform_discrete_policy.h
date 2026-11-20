#ifndef UNIFORM_DISCRETE_POLICY_H
#define UNIFORM_DISCRETE_POLICY_H

#include "bitrl/base/bitrl_types.h"

#include <utility>
#include <vector>

namespace bitrl
{
namespace rl
{
namespace policies
{

///
/// \brief The UniformDiscretePolicy class
///
class UniformDiscretePolicy final
{
  public:
    ///
    /// \brief UniformDiscretePolicy
    ///
    UniformDiscretePolicy(uint_t n_states, uint_t n_actions);

    ///
    /// \brief UniformDiscretePolicy
    ///
    UniformDiscretePolicy(uint_t n_states, uint_t n_actions, real_t val);

    ///
    /// \brief operator ()
    /// \param sidx
    /// \return
    ///
    std::vector<std::pair<uint_t, real_t>> operator()(uint_t sidx) const { return (*this)[sidx]; }

    ///
    /// \brief operator []
    ///
    std::vector<std::pair<uint_t, real_t>> operator[](uint_t sidx) const;

    ///
    /// \brief Update the policy for state with index sidx
    ///
    void update(uint_t sidx, const std::vector<std::pair<uint_t, real_t>> &vals);

    ///
    /// \brief equals
    ///
    bool equals(const UniformDiscretePolicy &other) const;

    ///
    /// \brief state_actions_values
    /// \return
    ///
    std::vector<std::vector<std::pair<uint_t, real_t>>> &state_actions_values()
    {
        return state_actions_prob_;
    }

    ///
    /// \brief shape
    /// \return
    ///
    std::pair<uint_t, uint_t> shape() const { return {n_states_, n_actions_}; }

    ///
    /// \brief update
    /// \param other
    ///
    void update(const UniformDiscretePolicy &other);

    ///
    /// \brief print
    /// \param out
    /// \return
    ///
    std::ostream &print(std::ostream &out) const;

  private:
    ///
    /// \brief n_states_
    ///
    uint_t n_states_;

    ///
    /// \brief n_actions_
    ///
    uint_t n_actions_;

    ///
    /// \brief val_
    ///
    real_t val_;

    ///
    /// \brief state_actions_prob_
    ///
    std::vector<std::vector<std::pair<uint_t, real_t>>> state_actions_prob_;

    ///
    /// \brief init_
    ///
    void init_();
};

inline bool operator==(const UniformDiscretePolicy &p1, const UniformDiscretePolicy &p2)
{
    return p1.equals(p2);
}

inline bool operator!=(const UniformDiscretePolicy &p1, const UniformDiscretePolicy &p2)
{
    return !(p1 == p2);
}

} // namespace policies
} // namespace rl
} // namespace bitrl

#endif // UNIFORM_DISCRETE_POLICY_H
