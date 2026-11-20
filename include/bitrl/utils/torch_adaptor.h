#ifndef TORCH_STATE_ADAPTOR_H
#define TORCH_STATE_ADAPTOR_H

#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_types.h"

#include <torch/torch.h>
#include <vector>

namespace bitrl
{
namespace utils
{
namespace pytorch
{

///
/// \brief The TorchStateAdaptor struct
///
struct TorchAdaptor
{

    typedef torch_tensor_t value_type;

    static value_type to_torch(const std::vector<real_t> &data, DeviceType dtype = DeviceType::CPU,
                               bool requires_grad = false);

    static value_type to_torch(const std::vector<uint_t> &data, DeviceType dtype = DeviceType::CPU,
                               bool requires_grad = false);

    static value_type to_torch(const std::vector<float_t> &data, DeviceType dtype = DeviceType::CPU,
                               bool requires_grad = false);
    static value_type to_torch(const std::vector<int_t> &data, DeviceType dtype = DeviceType::CPU,
                               bool requires_grad = false);
    static value_type to_torch(const std::vector<lint_t> &data, DeviceType dtype = DeviceType::CPU,
                               bool requires_grad = false);
    static value_type to_torch(const std::vector<bool> &data, DeviceType dtype = DeviceType::CPU,
                               bool requires_grad = false);

    template <typename T>
    static value_type stack(const std::vector<T> &values, DeviceType type = DeviceType::CPU,
                            bool requires_grad = false);

    //	static value_type stack(const std::vector<value_type>& values,
    //	                        DeviceType type=DeviceType::CPU)const;

    static value_type cat(const std::vector<real_t> &values, DeviceType type = DeviceType::CPU,
                          bool requires_grad = false);

    template <typename T>
    static value_type stack_as_float(const std::vector<std::vector<T>> &values,
                                     DeviceType type = DeviceType::CPU);

    template <typename T> static std::vector<T> to_vector(torch_tensor_t tensor);

    torch_tensor_t operator()(real_t value) const;
    torch_tensor_t operator()(const std::vector<real_t> &data) const;
    torch_tensor_t operator()(const std::vector<float_t> &data) const;
    torch_tensor_t operator()(const std::vector<int> &data) const;
};

template <typename T>
TorchAdaptor::value_type TorchAdaptor::stack_as_float(const std::vector<std::vector<T>> &values,
                                                      DeviceType dtype)
{

    std::vector<std::vector<float_t>> values_(values.size());

    // make the input values floats
    for (uint_t i = 0; i < values.size(); ++i)
    {
        values_[i].resize(values[i].size());
        for (uint_t j = 0; j < values[i].size(); ++j)
        {
            values_[i][j] = static_cast<float>(values[i][j]);
        }
    }

    return TorchAdaptor::stack(values_, dtype);
}

} // namespace pytorch
} // namespace utils
} // namespace bitrl
#endif
#endif // TORCH_STATE_ADAPTOR_H
