// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

#ifndef LEARNING_RATE_SCHEDULER_H
#define LEARNING_RATE_SCHEDULER_H
#include "bitrl/base/bitrl_types.h"

#include <algorithm>

namespace bitrl
{
namespace rl
{

/**
 * @brief struct ConstantLRScheduler. Echos bach the learning rate it was given
 * */
struct ConstantLRScheduler
{

    real_t operator()(real_t alpha, uint_t /*episode_idx*/) const { return alpha; }
};

/**
 * @brief struct InverseDecayLRScheduler. Decays the learning rate it was given
 * with the episodes, as alpha / (1 + episode / decay), but not below min_alpha.
 * A constant learning rate keeps the estimates of Monte Carlo and temporal
 * difference methods moving with every new return; a decaying one lets them
 * settle on the mean.
 * */
struct InverseDecayLRScheduler
{
    real_t decay{100.0};
    real_t min_alpha{0.001};

    real_t operator()(real_t alpha, uint_t episode_idx) const
    {
        return std::max(min_alpha, alpha / (1.0 + static_cast<real_t>(episode_idx) / decay));
    }
};

} // namespace rl
} // namespace bitrl

#endif // LEARNING_RATE_SCHEDULER_H
