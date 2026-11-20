// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

#ifndef LEARNING_RATE_SCHEDULER_H
#define LEARNING_RATE_SCHEDULER_H
#include "bitrl/base/bitrl_types.h"

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

} // namespace rl
} // namespace bitrl

#endif // LEARNING_RATE_SCHEDULER_H
