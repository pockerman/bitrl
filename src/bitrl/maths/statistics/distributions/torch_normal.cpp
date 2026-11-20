//
// Created by alex on 7/26/25.
//
#include "bitrl/maths/statistics//distributions/torch_normal.h"

#ifdef USE_PYTORCH

#include "torch/torch.h"
#include <cmath>
namespace bitrl
{
namespace maths::stats
{

TorchNormalDist::TorchNormalDist(torch_tensor_t mu, torch_tensor_t sigma) : mean_(mu), sd_(sigma) {}

torch_tensor_t TorchNormalDist::sample() const { return torch::randn_like(mean_) * sd_ + mean_; }

torch_tensor_t TorchNormalDist::log_prob(torch_tensor_t value)
{
    static const double log_sqrt_2pi = std::log(std::sqrt(2 * M_PI));
    torch::Tensor var = sd_ * sd_;
    return -((value - mean_).pow(2)) / (2 * var) - sd_.log() - log_sqrt_2pi;
}

torch_tensor_t TorchNormalDist::entropy() const
{
    static const real_t log_sqrt_2pie = 0.5 * std::log(2 * M_PI * std::exp(1));
    return torch::log(sd_) + log_sqrt_2pie;
}
} // namespace maths::stats
} // namespace bitrl

#endif
