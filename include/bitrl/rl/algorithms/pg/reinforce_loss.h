#ifndef REINFORCE_LOSS_H
#define REINFORCE_LOSS_H

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitsim/bitsim_consts.h"

#include <vector>

namespace bitrl
{
namespace rl
{
namespace algos
{
namespace pg
{

// using namespace bitsim::consts;

///
/// \brief compute_loss_item. Compute the product -rewards[i] * log_probs[i]
///
std::vector<torch_tensor_t> compute_loss_item(const std::vector<real_t> &rewards,
                                              const std::vector<torch_tensor_t> &log_probs);

///
/// \brief Compute the baseline using a user-defined constant
/// value from the given rewards
///
std::vector<real_t> compute_baseline_with_constant(const std::vector<real_t> &rewards,
                                                   real_t constant);

///
/// \brief Compute the baseline using the mean value of the
/// provided rewards
///
std::vector<real_t> compute_baseline_with_mean(const std::vector<real_t> &rewards);

///
/// \brief Compute the baseline using standarization also known as
/// whitening
///
std::vector<real_t> compute_baseline_with_standardization(const std::vector<real_t> &rewards,
                                                          real_t eps = bitsim::consts::TOLERANCE);

} // namespace pg
} // namespace algos
} // namespace rl
} // namespace bitrl

#endif
#endif
