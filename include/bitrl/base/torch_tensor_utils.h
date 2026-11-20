#ifndef TORCH_TENSOR_UTILS_H
#define TORCH_TENSOR_UTILS_H

#include "bitrl/base/cubeai_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/cubeai_types.h"
#include "bitrl/io/torch_state_dictionary_reader.h"
#include <torch/torch.h>

namespace cubeai
{
namespace torch_utils
{

///
/// \brief create_mask. Create a mask from the given container. The
/// contaier should contain values convertble to bool
///
template <typename ContainerType> torch_tensor_t create_mask(const ContainerType &container)
{

    return torch::tensor(container.data(), torch::dtype(torch::kBool));
}

///
///
///
template <typename TorchNetType>
void copy_parameters_to(/*TorchNetType& from,*/ TorchNetType &to, const std::string params_path)
{

    // make parameters copying possible
    torch::autograd::GradMode::set_enabled(false);

    //
    // from.save(params_path);

    auto new_params = TorchStateDictionaryReader(params_path); // implement this
    auto params = to->named_parameters(true /*recurse*/);
    auto buffers = to->named_buffers(true /*recurse*/);

    /*for (auto& val : new_params) {

         auto name = val.key();
         auto* t = params.find(name);

         if (t != nullptr) {
              t->copy_(val.value());
          }
         else {

              t = buffers.find(name);
              if (t != nullptr) {
                t->copy_(val.value());
              }
         }
    }*/

    torch::autograd::GradMode::set_enabled(true);
}

} // namespace torch_utils
} // namespace cubeai

#endif

#endif // TORCH_TENSOR_UTILS_H
