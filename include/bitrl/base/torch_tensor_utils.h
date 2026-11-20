#ifndef TORCH_TENSOR_UTILS_H
#define TORCH_TENSOR_UTILS_H

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include <torch/torch.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace bitrl
{
namespace torch_utils
{

///
/// \brief create_mask. Create a boolean mask from the given container. The
/// container should hold values convertible to bool
///
template <typename ContainerType> torch_tensor_t create_mask(const ContainerType &container)
{
    std::vector<std::int64_t> values;
    values.reserve(container.size());
    for (const auto &value : container)
    {
        values.push_back(static_cast<bool>(value) ? 1 : 0);
    }
    return torch::tensor(values, torch::dtype(torch::kInt64)).to(torch::kBool);
}

///
/// \brief copy_parameters. Copy the parameters and buffers of one network into
/// another of the same type, for example into the target network of DQN
/// \throws std::invalid_argument if `to` lacks a parameter or buffer of `from`
///
template <typename TorchNetType> void copy_parameters(const TorchNetType &from, TorchNetType &to)
{
    torch::NoGradGuard no_grad;

    auto params = to->named_parameters(true /*recurse*/);
    for (const auto &item : from->named_parameters(true /*recurse*/))
    {
        auto *param = params.find(item.key());
        if (param == nullptr)
        {
            throw std::invalid_argument("copy_parameters: no parameter " + item.key());
        }
        param->copy_(item.value());
    }

    auto buffers = to->named_buffers(true /*recurse*/);
    for (const auto &item : from->named_buffers(true /*recurse*/))
    {
        auto *buffer = buffers.find(item.key());
        if (buffer == nullptr)
        {
            throw std::invalid_argument("copy_parameters: no buffer " + item.key());
        }
        buffer->copy_(item.value());
    }
}

///
/// \brief copy_parameters_to. Copy into `to` the parameters and buffers of a
/// network of the same type that was saved with torch::save to params_path.
/// TorchNetType must be default constructible
///
template <typename TorchNetType>
void copy_parameters_to(TorchNetType &to, const std::string &params_path)
{
    TorchNetType from;
    torch::load(from, params_path);
    copy_parameters(from, to);
}

} // namespace torch_utils
} // namespace bitrl

#endif

#endif // TORCH_TENSOR_UTILS_H
