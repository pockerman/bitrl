#ifndef BITRL_RL_ALGORITHMS_ES_ES_CONFIG_H
#define BITRL_RL_ALGORITHMS_ES_ES_CONFIG_H

#include "bitrl/base/bitrl_types.h"

namespace bitrl
{
namespace rl::algos::es
{

///
/// \brief The settings of EvolutionStrategy
///
struct ESConfig
{
    uint_t n_pairs{32};            ///< antithetic pairs per generation: 2 * n_pairs evaluations
    real_t sigma{0.05};            ///< the standard deviation of the perturbations
    real_t learning_rate{0.02};    ///< Adam's step size
    real_t weight_decay{0.005};    ///< pulls the parameters towards 0, as an L2 penalty
    real_t beta1{0.9};             ///< Adam
    real_t beta2{0.999};           ///< Adam
    real_t adam_epsilon{1e-8};     ///< Adam
    bool rank_shaping{true};       ///< use centred ranks of the fitnesses, not the fitnesses
    uint_t seed{42};               ///< the seed of every perturbation
};

} // namespace rl::algos::es
} // namespace bitrl

#endif // BITRL_RL_ALGORITHMS_ES_ES_CONFIG_H
