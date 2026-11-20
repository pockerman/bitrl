// SPDX-FileCopyrightText: 2024 <copyright holder> <email>
// SPDX-License-Identifier: Apache-2.0

/**
 * Major implementation is taken from  https://github.com/Omegastick/pytorch-cpp-rl/tree/master
 *
 */

#include "bitrl/maths/statistics/distributions/torch_bernoulli_dist.h"

#ifdef USE_PYTORCH

#include <cmath>
#include <stdexcept>

namespace bitrl
{
namespace maths
{
namespace stats
{

TorchBernoulliDist::TorchBernoulliDist(torch_tensor_t probs, bool do_build_from_logits)
    :

      probs_(), logits_(), param_(), num_events_()
{
    if (do_build_from_logits)
    {
        build_from_logits(probs);
    }
    else
    {
        build_from_probabilities(probs);
    }
}

torch_tensor_t TorchBernoulliDist::entropy() const
{
    return torch::binary_cross_entropy_with_logits(logits_, probs_, torch::Tensor(),
                                                   torch::Tensor(), torch::Reduction::None);
}

torch_tensor_t TorchBernoulliDist::log_prob(torch_tensor_t value)
{
    auto broadcasted_tensors = torch::broadcast_tensors({logits_, value});
    return -torch::binary_cross_entropy_with_logits(broadcasted_tensors[0], broadcasted_tensors[1],
                                                    torch::Tensor(), torch::Tensor(),
                                                    torch::Reduction::None);
}

torch_tensor_t TorchBernoulliDist::sample(c10::ArrayRef<int64_t> sample_shape)
{
    auto ext_sample_shape = extended_shape(sample_shape);
    torch::NoGradGuard no_grad_guard;
    return torch::bernoulli(probs_.expand(ext_sample_shape));
}

void TorchBernoulliDist::build_from_logits(torch_tensor_t logits)
{

    if (logits.dim() < 1)
    {
        throw std::runtime_error("Logits tensor must have at least one dimension");
    }

    logits_ = logits;
    probs_ = torch::sigmoid(logits_);

    param_ = logits;
    num_events_ = param_.size(-1);
    batch_shape_ = param_.sizes().vec();
}

void TorchBernoulliDist::build_from_probabilities(torch_tensor_t probs)
{

    if (probs.dim() < 1)
    {
        throw std::runtime_error("Probabilities tensor must have at least one dimension");
    }

    probs_ = probs;

    auto clamped_probs = probs_.clamp(1.21e-7, 1. - 1.21e-7);
    logits_ = torch::log(clamped_probs) - torch::log1p(-clamped_probs);

    param_ = probs_;
    num_events_ = param_.size(-1);
    batch_shape_ = param_.sizes().vec();
}

} // namespace stats
} // namespace maths
} // namespace bitrl

#endif
