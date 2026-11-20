#ifndef UTILS_H
#define UTILS_H

///
/// \brief Various utilities used when working with RL
/// problems
///

#include "bitrl/base/bitrl_types.h"
#include "bitrl/maths/vector_math.h"

#include <vector>

namespace bitrl
{
namespace rl::algos
{

///
/// \brief Given the state index returns the list of actions under the
/// provided value functions
///
template <typename WorldTp>
auto state_actions_from_v(const WorldTp &env, const DynVec<real_t> &v, real_t gamma, uint_t state)
    -> DynVec<real_t>
{

    auto q = DynVec<real_t>(env.n_actions());
    std::for_each(q.begin(), q.end(), [](auto &item) { item = 0.0; });

    for (uint_t a = 0; a < env.n_actions(); ++a)
    {

        const auto &transition_dyn = env.p(state, a);

        for (auto &dyn : transition_dyn)
        {
            auto prob = std::get<0>(dyn);
            auto next_state = std::get<1>(dyn);
            auto reward = std::get<2>(dyn);
            // auto done = std::get<3>(dyn);
            q[a] += prob * (reward + gamma * v[next_state]);
        }
    }

    return q;
}

///
/// \brief create_discounts_array
/// \param end
/// \param base
/// \param start
/// \param endpoint
/// \return
///
template <typename T> std::vector<T> create_discounts_array(T base, uint_t npoints)
{
    std::vector<T> points(npoints, 0.0);
    for (uint_t i = 0; i < npoints; ++i)
    {
        points[i] = std::pow(base, i);
    }
    return points;
}

///
/// \brief Create a vector where element i  is the product
///  $$\gamma^i * rewards[i]$$
///
template <typename T>
std::vector<T> calculate_discounted_return_vector(const std::vector<T> &rewards, T gamma)
{
    std::vector<T> returns(rewards.size(), 0.0);
    for (uint_t t = 0; t < rewards.size(); ++t)
    {
        returns[t] = std::pow(gamma, t) * rewards[t];
    }

    return returns;
}

///
/// \brief calculate_discounted_return_vector. Creates the discounted return vector
/// for the given trajectory
///
template <typename TimeStepType, typename T>
std::vector<T> calculate_discounted_return_vector(const std::vector<TimeStepType> &trajectory,
                                                  T gamma)
{

    std::vector<T> rewards(trajectory.size());
    for (uint_t t = 0; t < trajectory.size(); ++t)
    {
        rewards[t] = std::pow(gamma, t) * trajectory[t].reward();
    }

    return rewards;
}

///
/// \brief calculate_discounted_return. Calculates the sum of the discounted
/// rewards for the given rewards array using the given gamma
/// \param rewards
/// \param gamma
/// \return
///
template <typename T> T calculate_discounted_return(const std::vector<T> &rewards, T gamma)
{
    auto discounted_vector = calculate_discounted_return_vector(rewards, gamma);
    return bitrl::maths::sum(discounted_vector);
}

///
/// \brief calculate_mean_discounted_return. Same as calculate_discounted_return
/// but the result is weighted by 1/N where N is the size of the given rewards array
/// \param rewards
/// \param gamma
/// \return
///
template <typename T> T calculate_mean_discounted_return(const std::vector<T> &rewards, T gamma)
{
    auto discounted_vector = calculate_discounted_return_vector(rewards, gamma);
    return bitrl::maths::mean(discounted_vector);
}

///
/// \brief Calculate the discounted return from the given trajectory
///
template <typename TimeStepType, typename T>
T calculate_discounted_return(const std::vector<TimeStepType> &trajectory, T gamma)
{

    auto discounted_vector = calculate_discounted_return_vector(trajectory, gamma);
    return bitrl::maths::sum(discounted_vector);
}

template <typename TimeStepType, typename T>
T calculate_mean_discounted_return(const std::vector<TimeStepType> &trajectory, T gamma)
{
    auto discounted_vector = calculate_discounted_return_vector(trajectory, gamma);
    return bitrl::maths::mean(discounted_vector);
}

///
/// \brief Given an array of rewards, for each entry calculate the
/// following:
/// $$G = \sum_{k=t+1}^T \gamma^{k-t-1}R_k$$
///
template <typename T>
std::vector<T> calculate_step_discounted_return(const std::vector<T> &rewards, T gamma)
{
    std::vector<T> discounted_returns(rewards.size());
    for (uint_t t = 0; t < rewards.size(); ++t)
    {

        T G = 0.0;
        auto begin = rewards.begin();
        // advance the iterator t positions
        std::advance(begin, t);
        auto counter = 0;
        for (; begin != rewards.end(); ++begin)
        {
            G += std::pow(gamma, counter++) * (*begin);
        }

        discounted_returns[t] = G;
    }

    return discounted_returns;
}
} // namespace rl::algos
} // namespace bitrl

#endif // UTILS_H
