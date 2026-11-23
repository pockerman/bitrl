#ifndef BITRL_RL_DISTRIBUTED_TORCH_PARAMETERS_H
#define BITRL_RL_DISTRIBUTED_TORCH_PARAMETERS_H

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"
#include "bitrl/rl/distributed/communicator.h"

#include <torch/torch.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace bitrl
{
namespace rl::distributed
{

namespace detail
{
inline void check_float_(const torch::Tensor &t)
{
    if (t.scalar_type() != torch::kFloat32)
    {
        throw std::invalid_argument("bitrl::rl::distributed: the parameters must be float32");
    }
}

template <typename Get>
std::vector<float> flatten_(const std::vector<torch::Tensor> &tensors, Get get)
{
    std::vector<float> flat;
    for (const auto &p : tensors)
    {
        check_float_(p);
        const auto t = get(p);
        const auto n = static_cast<std::size_t>(p.numel());
        if (!t.defined())
        {
            flat.insert(flat.end(), n, 0.0f);
            continue;
        }
        const auto c = t.detach().contiguous();
        flat.insert(flat.end(), c.template data_ptr<float>(), c.template data_ptr<float>() + n);
    }
    return flat;
}
} // namespace detail

///
/// \brief The values of the parameters, one after the other
///
inline std::vector<float> flatten_parameters(const std::vector<torch::Tensor> &parameters)
{
    return detail::flatten_(parameters, [](const torch::Tensor &p) { return p; });
}

///
/// \brief The gradients of the parameters, one after the other; 0 where a parameter has none
///
inline std::vector<float> flatten_gradients(const std::vector<torch::Tensor> &parameters)
{
    return detail::flatten_(parameters, [](const torch::Tensor &p) { return p.grad(); });
}

///
/// \brief Copy values, as flatten_parameters() gives them, into the parameters
///
inline void set_parameters(const std::vector<torch::Tensor> &parameters,
                           const std::vector<float> &values)
{
    torch::NoGradGuard no_grad;
    std::size_t at = 0;
    for (auto p : parameters)
    {
        const auto n = static_cast<std::size_t>(p.numel());
        if (at + n > values.size())
        {
            throw std::invalid_argument("set_parameters: too few values");
        }
        const auto src = torch::from_blob(const_cast<float *>(values.data() + at), p.sizes(),
                                          torch::TensorOptions().dtype(torch::kFloat32));
        p.copy_(src);
        at += n;
    }
    if (at != values.size())
    {
        throw std::invalid_argument("set_parameters: too many values");
    }
}

///
/// \brief Copy values, as flatten_gradients() gives them, into the gradients
///
inline void set_gradients(const std::vector<torch::Tensor> &parameters,
                          const std::vector<float> &values)
{
    std::size_t at = 0;
    for (auto p : parameters)
    {
        const auto n = static_cast<std::size_t>(p.numel());
        if (at + n > values.size())
        {
            throw std::invalid_argument("set_gradients: too few values");
        }
        const auto src = torch::from_blob(const_cast<float *>(values.data() + at), p.sizes(),
                                          torch::TensorOptions().dtype(torch::kFloat32))
                             .clone();
        if (p.mutable_grad().defined())
        {
            p.mutable_grad().copy_(src);
        }
        else
        {
            p.mutable_grad() = src;
        }
        at += n;
    }
    if (at != values.size())
    {
        throw std::invalid_argument("set_gradients: too many values");
    }
}

///
/// \brief Replace the gradients of every rank with their mean over the ranks
///
inline void average_gradients(Communicator &communicator,
                              const std::vector<torch::Tensor> &parameters)
{
    auto g = flatten_gradients(parameters);
    communicator.all_reduce_sum(g);
    const float n = static_cast<float>(communicator.world_size());
    for (auto &v : g)
    {
        v /= n;
    }
    set_gradients(parameters, g);
}

///
/// \brief Give every rank the parameters of root
///
inline void broadcast_parameters(Communicator &communicator,
                                 const std::vector<torch::Tensor> &parameters, uint_t root = 0)
{
    auto values = flatten_parameters(parameters);
    communicator.broadcast(values, root);
    set_parameters(parameters, values);
}

///
/// \brief A checksum of the parameters, to tell that the ranks have the same
///
inline std::uint64_t parameters_checksum(const std::vector<torch::Tensor> &parameters)
{
    const auto values = flatten_parameters(parameters);
    std::uint64_t h = 1469598103934665603ULL; // FNV-1a
    for (const float v : values)
    {
        std::uint32_t bits;
        std::memcpy(&bits, &v, sizeof(bits));
        for (int b = 0; b < 32; b += 8)
        {
            h ^= (bits >> b) & 0xFFu;
            h *= 1099511628211ULL;
        }
    }
    return h;
}

} // namespace rl::distributed
} // namespace bitrl

#endif // USE_PYTORCH
#endif // BITRL_RL_DISTRIBUTED_TORCH_PARAMETERS_H
