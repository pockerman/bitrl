#ifndef RL_MIXINS_H
#define RL_MIXINS_H
/**
  * Utility class used in the implementation
  * of RL algorithms
  * */
#include "bitrl/base/bitrl_config.h"
#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/epsilon_decay_options.h"


#ifdef BITRL_DEBUG
#include <cassert>
#endif

#include <map>
#include <tuple>
#include <random>


namespace bitrl::rl{

namespace  {

template<typename StateTp>
const DynVec<real_t>&
get_table_values_(const std::map<StateTp,DynVec<real_t>>& table, const StateTp& state ){

    auto itr = table.find(state);
#ifdef CUBEAI_DEBUG
    if(itr == table.end()){
        assert(false && "Invalid state given");
    }
#endif

    return itr->second;

}

template<typename StateTp>
DynVec<real_t>&
get_table_values_(std::map<StateTp,DynVec<real_t>>& table, const StateTp& state ){

    auto itr = table.find(state);
#ifdef BITRL_DEBUG
    if(itr == table.end()){
        assert(false && "Invalid state given");
    }
#endif

    return itr->second;

}

}

///
/// \brief max_action
/// \param qtable
/// \param state
/// \param n_actions
/// \return
///
uint_t max_action(const DynMat<real_t>& qtable, uint_t state, uint_t n_actions);


///
/// \brief The with_decay_epsilon_option_mixin struct
///
struct with_decay_epsilon_option_mixin
{
    real_t eps_init;
    real_t eps;
    real_t min_eps;
    real_t max_eps;
    real_t epsilon_decay;
    uint_t n_actions;
    uint_t seed;
    EpsilonDecayOptionType decay_op;

    ///
    /// \brief decay_eps
    /// \param episode_index
    /// \return
    ///
    real_t decay_eps(uint_t episode_index);

    ///
    ///
    ///
    template<typename VectorType>
    uint_t choose_action_index(const VectorType& values)const;
};

template<typename VectorType>
uint_t
with_decay_epsilon_option_mixin::choose_action_index(const VectorType& values)const{

    std::mt19937 gen(this->with_decay_epsilon_option_mixin::seed);

    // generate a number in [0, 1]
    std::uniform_real_distribution<> real_dist_(0.0, 1.0);

    if(real_dist_(gen) > this->with_decay_epsilon_option_mixin::eps){
        // select greedy action with probability 1 - epsilon
        return arg_max(values);
    }

    std::uniform_int_distribution<> distrib_(0,  this->with_decay_epsilon_option_mixin::n_actions - 1);
    return distrib_(gen);

}

///
/// \brief The WithQTableMixin struct
///
struct with_q_table_mixin
{
    typedef uint_t state_type;
    typedef uint_t action_type;
    typedef real_t value_type;

    ///
    /// \brief q_table
    ///
    DynMat<value_type> q_table;

    ///
    /// \brief initialize
    /// \param n_states
    /// \param n_actions
    /// \param init_value
    ///
    void initialize(state_type n_states, action_type n_actions, real_t init_value);
};

template<typename TableTp>
struct with_double_q_table_mixin;

///
/// \brief The WithDoubleQTableMixin struct
///
template<>
struct with_double_q_table_mixin< DynMat<real_t> >
{
    typedef uint_t index_type;
    typedef uint_t state_type;
    typedef uint_t action_type;
    typedef real_t value_type ;

    ///
    /// \brief q_table_1
    ///
    DynMat<value_type> q_table_1;

    ///
    /// \brief q_table_2
    ///
    DynMat<value_type> q_table_2;

    ///
    /// \brief initialize
    /// \param n_states
    /// \param n_actions
    /// \param init_value
    ///
    void initialize(const std::vector<index_type>& indices, action_type n_actions, real_t init_value);

    ///
    ///
    ///
    template<int index>
    value_type get(const state_type& state, const action_type action)const;

    template<int index>
    void set(const state_type& state, const action_type action, const value_type value);
};

template<>
with_double_q_table_mixin< DynMat<real_t>>::value_type
with_double_q_table_mixin< DynMat<real_t>>::get<1>(const state_type& state, const action_type action)const{
    return q_table_1(state, action);
}

template<>
with_double_q_table_mixin< DynMat<real_t>>::value_type
with_double_q_table_mixin< DynMat<real_t>>::get<2>(	const state_type& state, 
													const action_type action)const{
    return q_table_2(state, action);
}

template<>
void
with_double_q_table_mixin< DynMat<real_t>>::set<1>(	const state_type& state, 
													const action_type action, 
													const value_type value){
    q_table_1(state, action) = value;
}

template<>
void
with_double_q_table_mixin< DynMat<real_t>>::set<2>(	const state_type& state, 
													const action_type action, 
													const value_type value){
    q_table_2(state, action) = value;
}




template<typename KeyTp>
struct with_double_q_table_mixin<std::map<KeyTp, DynVec<real_t>>>
{

    typedef KeyTp index_type;
    typedef KeyTp state_type;
    typedef uint_t action_type;
    typedef real_t value_type ;

    ///
    /// \brief q_table_1
    ///
    std::map<KeyTp, DynVec<real_t>> q_table_1;

    ///
    /// \brief q_table_2
    ///
    std::map<KeyTp, DynVec<real_t>> q_table_2;

    ///
    /// \brief initialize
    /// \param n_states
    /// \param n_actions
    /// \param init_value
    ///
    void initialize(const std::vector<index_type>& indices, action_type n_actions, real_t init_value);

    ///
    ///
    ///
    template<int index>
    value_type get(const state_type& state, const action_type action)const;

    ///
    ///
    ///
    template<int index>
    void set(const state_type& state, const action_type action, const value_type value);

};

template<typename KeyTp>
void
with_double_q_table_mixin<std::map<KeyTp, DynVec<real_t>>>::initialize(	const std::vector<index_type>& indices, 
																		action_type n_actions, 
																		real_t init_value){


    DynVec<real_t> init_vals(n_actions, init_value);

    for(uint_t i=0; i< indices.size(); ++i){

        q_table_1[indices[i]] = init_vals;
        q_table_2[indices[i]] = init_vals;
    }
}

template<typename KeyTp>
template<int index>
typename with_double_q_table_mixin<std::map<KeyTp, DynVec<real_t>>>::value_type
with_double_q_table_mixin<std::map<KeyTp, DynVec<real_t>>>::get(const state_type& state, const action_type action)const{

    static_assert (index == 1 || index == 2, "Invalid index for template parameter");
    if(index == 1){
        return get_table_values_(q_table_1, state)[action];
    }

    return get_table_values_(q_table_2, state)[action];

}

template<typename KeyTp>
template<int index>
void
with_double_q_table_mixin<std::map<KeyTp, DynVec<real_t>>>::set(const state_type& state, 
																const action_type action, 
																const value_type value){

    static_assert (index == 1 || index == 2, "Invalid index for template parameter");

    if(index == 1){
        auto& vals1 = get_table_values_(q_table_1, state);
        vals1[action] = value;
    }

    auto& vals2 = get_table_values_(q_table_2, state);
    vals2[action] = value;
}


struct with_double_q_table_max_action_mixin
{

    ///
    /// \brief Returns the max action by averaging the state values from the two tables
    ///
    template<typename TableTp, typename StateTp>
    static uint_t max_action(const TableTp& q1_table, const TableTp& q2_table,
                             const StateTp& state, uint_t n_actions);

    ///
    /// \brief Returns the max action by averaging the state values from the two tables
    ///
    template<typename TableTp, typename StateTp>
    static uint_t max_action(const TableTp& q1_table,  const StateTp& state, uint_t n_actions);

};


template<typename TableTp, typename StateTp>
uint_t
with_double_q_table_max_action_mixin::max_action(const TableTp& q1_table, const TableTp& q2_table,
                                                 const StateTp& state, uint_t /*n_actions*/){

   const auto& vals1 = get_table_values_(q1_table, state);
   const auto& vals2 = get_table_values_(q2_table, state);
   auto sum   = vals1 + vals2;
   return 1; //blaze::argmax(sum);

}

template<typename TableTp, typename StateTp>
uint_t
with_double_q_table_max_action_mixin::max_action(const TableTp& q_table, const StateTp& state, uint_t /*n_actions*/){

   const auto& vals = get_table_values_(q_table, state);
   return 1; //blaze::argmax(vals);

}



}

#endif // RL_MIXINS_H
