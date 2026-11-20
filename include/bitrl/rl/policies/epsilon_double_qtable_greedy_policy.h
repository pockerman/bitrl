#ifndef EPSILON_DOUBLE_QTABLE_GREEDY_POLICY_H
#define EPSILON_DOUBLE_QTABLE_GREEDY_POLICY_H

#include "cubeai/base/cubeai_types.h"
#include "cubeai/rl/epsilon_decay_options.h"
#include "cubeai/rl/rl_mixins.h"

namespace cubeai {
namespace rl {
namespace policies {

///
/// \brief
///
template<typename TableType>
class EpsilonDoubleQTableGreedyPolicy: protected with_decay_epsilon_option_mixin, protected with_double_q_table_max_action_mixin
{
public:

    ///
    /// \brief table_type
    ///
    typedef TableType table_type;

    ///
    ///
    ///
    using with_decay_epsilon_option_mixin::choose_action_index;

    ///
    /// \brief EpsilonDoubleQTableGreedyPolicy
    ///
    explicit EpsilonDoubleQTableGreedyPolicy(real_t eps, uint_t n_actions,
                                             EpsilonDecayOptionType decay_op,
                                             real_t min_eps = 0.01, real_t max_eps=1.0,
                                             real_t eps_decay=0.2, uint_t seed=0);

    ///
    /// \brief operator()
    ///
    template<typename StateTp>
    uint_t operator()(const TableType& q1, const TableType& q2, const StateTp& state)const;

    ///
    /// \brief choose_action_index
    ///
    //template<typename VectorType>
    //uint_t choose_action_index(const VectorType& values)const;

    ///
    /// \brief adjust_on_episode
    /// \param episode
    ///
    void adjust_on_episode(uint_t episode)noexcept;

    ///
    /// \brief reset
    ///
    void reset()noexcept{this->with_decay_epsilon_option_mixin::eps = this->with_decay_epsilon_option_mixin::eps_init;}

    ///
    /// \brief set_epsilon_decay_factor
    /// \param eps_decay
    ///
    void set_epsilon_decay_factor(real_t eps_decay)noexcept{this->with_decay_epsilon_option_mixin::epsilon_decay = eps_decay;}

    ///
    /// \brief eps_value
    ///
    real_t eps_value()const noexcept{return this->with_decay_epsilon_option_mixin::eps;}

    ///
    /// \brief set_seed
    /// \param seed
    ///
    void set_seed(const uint_t seed)noexcept{this->with_decay_epsilon_option_mixin::seed = seed;}

};

template<typename TableType>
EpsilonDoubleQTableGreedyPolicy<TableType>::EpsilonDoubleQTableGreedyPolicy(real_t eps, uint_t n_actions,
                                                                            EpsilonDecayOptionType decay_op,
                                                                            real_t min_eps, real_t max_eps, real_t eps_decay,  uint_t seed)
    :
      with_decay_epsilon_option_mixin({eps, eps, min_eps, max_eps, eps_decay, n_actions, seed, decay_op})
{}

template<typename TableType>
template<typename StateTp>
uint_t
EpsilonDoubleQTableGreedyPolicy<TableType>::operator()(const TableType& q1, const TableType& q2, const StateTp& state)const{


    std::mt19937 gen(this->with_decay_epsilon_option_mixin::seed);

    // generate a number in [0, 1]
    std::uniform_real_distribution<> real_dist_(0.0, 1.0);

    if(real_dist_(gen) > this->with_decay_epsilon_option_mixin::eps){
        // select greedy action with probability 1 - epsilon
        return this->with_double_q_table_max_action_mixin::max_action(q1, q2, state,
                                                                      this->with_decay_epsilon_option_mixin::n_actions);
    }

    std::uniform_int_distribution<> distrib_(0,  this->with_decay_epsilon_option_mixin::n_actions - 1);
    return distrib_(gen);
}

template<typename TableType>
void
EpsilonDoubleQTableGreedyPolicy<TableType>::adjust_on_episode(uint_t episode)noexcept{
    this->with_decay_epsilon_option_mixin::eps = this->with_decay_epsilon_option_mixin::decay_eps(episode);
}

/*template<typename TableType>
template<typename VectorType>
uint_t
EpsilonDoubleQTableGreedyPolicy<TableType>::choose_action_index(const VectorType& values)const{

    std::mt19937 gen(this->with_decay_epsilon_option_mixin::seed);

    // generate a number in [0, 1]
    std::uniform_real_distribution<> real_dist_(0.0, 1.0);

    if(real_dist_(gen) > this->with_decay_epsilon_option_mixin::eps){
        // select greedy action with probability 1 - epsilon
        return arg_max(values);
    }

    std::uniform_int_distribution<> distrib_(0,  this->with_decay_epsilon_option_mixin::n_actions - 1);
    return distrib_(gen);

}*/

}

}

}

#endif // EPSILON_DOUBLE_QTABLE_GREEDY_POLICY_H
