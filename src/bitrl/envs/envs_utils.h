// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

#ifndef ENVS_UTILS_H
#define ENVS_UTILS_H

#include "bitrl/bitrl_config.h"
#include "bitrl/bitrl_types.h"

#include <vector>

#ifdef BITRL_DEBUG
#include <cassert>
#endif

namespace bitrl
{
namespace envs
{

/**
 * @todo write docs
 */

template <typename EnvType, typename ActionSelector>
std::vector<typename EnvType::time_step_type>
create_trajectory(EnvType &env, ActionSelector &action_selector, uint_t max_steps)
{

#ifdef BITRL_DEBUG
    assert(max_steps >= 1 && "Attempt to generate environment trajector with max_steps < 1");
#endif

    typedef typename EnvType::time_step_type time_step_type;

    std::vector<time_step_type> trajectory;
    trajectory.reserve(max_steps);

    auto done = false;

    while (!done)
    {
        auto time_step = env.reset();

        // loop infinitely
        uint_t counter = 0;
        while (true)
        {

            // select the action given the observation
            auto action = action_selector(time_step.observation());

            auto new_time_step = env.step(action);
            done = new_time_step.done();
            trajectory.push_back(new_time_step);

            if (new_time_step.done())
            {
                break;
            }

            if (counter >= max_steps - 1)
            {
                trajectory.clear();
                break;
            }

            time_step = new_time_step;
        }
    }

    return trajectory;
}

} // namespace envs
} // namespace bitrl

#endif // ENVS_UTILS_H
