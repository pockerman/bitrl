#include "bitrl/rl/rl_mixins.h"
#include <cmath>

namespace bitrl::rl{

real_t
with_decay_epsilon_option_mixin::decay_eps(uint_t episode_index){


    switch(decay_op){

        case EpsilonDecayOptionType::NONE:
        {
            break;
        }
        case EpsilonDecayOptionType::INVERSE_STEP:
        {
            if(episode_index == 0){
                episode_index = 1;
            }

            // there are various methods to do epsilon
            // reduction
            eps= 1.0 / episode_index;

            if(eps < min_eps){
                eps = min_eps;
            }
            break;
        }
        case EpsilonDecayOptionType::EXPONENTIAL:
        {
             eps = min_eps + (max_eps - min_eps)*std::exp(-epsilon_decay * episode_index);
             break;
        }
        case EpsilonDecayOptionType::CONSTANT_RATE:
        {
            eps -= epsilon_decay;

            if(eps < min_eps){
                eps = min_eps;
            }
            break;
        }
        default:
        {
            throw std::logic_error("Invalid decay type");
        }
    }

    return eps;

}

uint_t
max_action(const DynMat<real_t>& qtable, uint_t state, uint_t n_actions){

    DynVec<real_t> values(n_actions);

    for(uint_t a=0; a<n_actions; ++a){
        values[a] = qtable(state, a);
    }

    Eigen::Index minRow, minCol;
    values.maxCoeff(&minRow, &minCol);
    return minCol;
}

void
with_q_table_mixin::initialize(state_type n_states, action_type n_actions, real_t init_value){

    q_table.resize(n_states, n_actions);


    for(uint_t s=0; s< n_states; ++s){
        for(uint_t a=0; a< n_actions; ++a){
            q_table(s, a) = init_value;
        }
    }
}


void
with_double_q_table_mixin< DynMat<real_t> >::with_double_q_table_mixin::initialize(const std::vector<index_type>& indices,
                                                                                   action_type n_actions, real_t init_value){

    q_table_1.resize(indices.size(), n_actions);
    q_table_2.resize(indices.size(), n_actions);

    for(uint_t s=0; s< indices.size(); ++s){
        for(uint_t a=0; a < n_actions; ++a){

            q_table_1(s, a) = init_value;
            q_table_2(s, a) = init_value;
        }
    }
}


}
