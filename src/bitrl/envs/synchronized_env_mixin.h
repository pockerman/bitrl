// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

#ifndef SYNCHRONIZED_ENV_MIXIN_H
#define SYNCHRONIZED_ENV_MIXIN_H


#include <any>
#include <string>
#include <unordered_map>

namespace bitrl{
namespace envs{

/**
 * @brief class synchronized_env_mixin
 */
struct synchronized_env_mixin
{
    /**
     * @brief Synchronize the environment
     *
     */
    virtual void sync(const std::unordered_map<std::string, std::any>& /*options*/){}
};

}
}
#endif // SYNCHRONIZED_ENV_MIXIN_H
