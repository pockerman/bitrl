#include "bitrl/rl/policies/uniform_discrete_policy.h"
#include "bitrl/rl/policies/policy_type.h"

#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <unordered_map>

namespace bitrl
{
namespace rl
{
namespace policies
{

namespace
{

bool operator==(const std::pair<uint_t, uint_t> &s1, const std::pair<uint_t, uint_t> &s2)
{

    return s1.first == s2.first && s1.second == s2.second;
}

bool operator!=(const std::pair<uint_t, uint_t> &s1, const std::pair<uint_t, uint_t> &s2)
{

    return !(s1 == s2);
}

} // namespace

UniformDiscretePolicy::UniformDiscretePolicy(uint_t n_states, uint_t n_actions)
    : // DiscretePolicyBase(PolicyType::DISCRETE_UNIFORM),
      n_states_(n_states), n_actions_(n_actions), val_(-1.0), state_actions_prob_()
{
    val_ = 1.0 / static_cast<real_t>(n_actions_);
    init_();
}

void UniformDiscretePolicy::update(const UniformDiscretePolicy &other)
{
    state_actions_prob_ = other.state_actions_prob_;
}

UniformDiscretePolicy::UniformDiscretePolicy(uint_t n_states, uint_t n_actions, real_t val)
    : // DiscretePolicyBase(PolicyType::DISCRETE_UNIFORM),
      n_states_(n_states), n_actions_(n_actions), val_(val), state_actions_prob_()
{
    init_();
}

std::vector<std::pair<uint_t, real_t>> UniformDiscretePolicy::operator[](uint_t sidx) const
{

    return state_actions_prob_[sidx];
}

void UniformDiscretePolicy::update(uint_t sidx, const std::vector<std::pair<uint_t, real_t>> &vals)
{

#ifdef BITRL_DEBUG
    assert(sidx < n_states_ && "Invalid state index. Index must be < n_states_");
    assert(!vals.empty() && "Empty values to update state status are not allwed");
#endif

    auto &view = state_actions_prob_[sidx];

    // collect all the actions in a map
    auto act_val_map = std::unordered_map<uint_t, real_t>();

    for (uint_t a = 0; a < vals.size(); ++a)
    {
        act_val_map.insert({vals[a].first, vals[a].second});
    }

    for (uint_t a = 0; a < view.size(); ++a)
    {
        auto action = view[a].first;

        if (act_val_map.contains(action))
        {
            view[a].second = act_val_map[action];
        }
        /*else{
            view[a].second = 0.0;
        }*/
    }

    // state_actions_prob_[sidx] = vals;
}

bool UniformDiscretePolicy::equals(const UniformDiscretePolicy &other) const
{

    /*if(other.type() != PolicyType::DISCRETE_UNIFORM){
        return false;
    }*/

    // const auto& pol = dynamic_cast<const UniformDiscretePolicy&>(other);

    if (shape() != other.shape())
    {
        return false;
    }

    if (val_ != other.val_)
    {
        return false;
    }

    // check individual values
    // bool are_equal = true;

    for (uint_t s = 0; s < state_actions_prob_.size(); ++s)
    {
        for (uint_t a = 0; a < state_actions_prob_[s].size(); ++a)
        {

            auto aidx = state_actions_prob_[s][a].first;
            auto value = state_actions_prob_[s][a].second;

            if (aidx != other.state_actions_prob_[s][a].first)
            {
                return false;
            }

            if (value != other.state_actions_prob_[s][a].second)
            {
                return false;
            }
        }
    }

    return true;
}

void UniformDiscretePolicy::init_()
{

    state_actions_prob_.resize(n_states_);

    for (uint_t s = 0; s < n_states_; ++s)
    {
        state_actions_prob_[s].resize(n_actions_);

        for (uint_t a = 0; a < n_actions_; ++a)
        {
            state_actions_prob_[s][a] = {a, val_};
        }
    }
}

std::ostream &UniformDiscretePolicy::print(std::ostream &out) const
{

    for (uint_t s = 0; s < state_actions_prob_.size(); ++s)
    {
        for (uint_t a = 0; a < state_actions_prob_[s].size(); ++a)
        {

            if (a == state_actions_prob_[s].size() - 1)
            {
                out << state_actions_prob_[s][a].second << "\n";
            }
            else
            {
                out << state_actions_prob_[s][a].second << ",";
            }
        }
    }

    return out;
}

} // namespace policies

} // namespace rl
} // namespace bitrl
