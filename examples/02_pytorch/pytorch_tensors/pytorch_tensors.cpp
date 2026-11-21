#include "bitrl/base/bitrl_config.h"

#ifdef USE_PYTORCH

#include "bitrl/base/bitrl_log.h"
#include "bitrl/base/bitrl_types.h"

#include <torch/torch.h>

#include <iostream>
#include <string>
#include <typeinfo>
#include <vector>

namespace example2
{

using bitrl::int_t;
using bitrl::real_t;
using bitrl::uint_t;
} // namespace example2

int main()
{

    using namespace example2;

    if (torch::cuda::is_available())
    {
        std::cout << "CUDA is available on this machine" << std::endl;
    }
    else
    {
        std::cout << "CUDA is not available on this machine" << std::endl;
    }

    // create a torch::Tensor from a scalar value
    torch::Tensor t = torch::tensor(2.0);

    // the return value is c10::Scalar
    auto val = t.item();

    // use c10::Sclar.to<T>() to get the actual value
    // T is the actual type we want
    std::cout << "Tensor t is: " << val.to<float>() << std::endl;
    std::cout << " Dimension of tensor: " << t.dim() << std::endl;

    // various methods to create a tensor
    torch::Tensor tensor = torch::eye(3);
    std::cout << "Tensor is: " << tensor << std::endl;

    // floating point tensor
    std::vector<real_t> data(3, 2.0);
    auto tensor_from_data_1 = torch::tensor(data);

    std::cout << "Dimension of tensor: " << tensor_from_data_1.dim() << std::endl;

    // loop over the values of the tensor
    for (uint_t i = 0; i < static_cast<uint_t>(tensor_from_data_1.size(0)); ++i)
    {
        std::cout << tensor_from_data_1[i] << std::endl;
    }

    data[0] = data[1] = data[2] = 1.0;
    auto tensor_from_data_2 = torch::tensor(data);
    std::cout << tensor_from_data_2 << std::endl;

    // integer tensor
    std::vector<int_t> uint_data = {1, 2, 3, 4};
    auto tensor_from_data_3 = torch::tensor(uint_data);

    // unsqueeze operation
    auto unsqueezed_tensor = tensor_from_data_3.unsqueeze(0);

    // add two tensors together
    auto sum = tensor_from_data_2 + tensor_from_data_1;
    std::cout << sum << std::endl;

    // difference of two tensors
    auto diff = tensor_from_data_2 - tensor_from_data_1;
    std::cout << sum << std::endl;

    // compute element-wise product
    auto tensor1 = torch::tensor({1.0, 2.0, 3.0});
    auto product = tensor1 * tensor1;
    std::cout << product << std::endl;

    if (torch::cuda::is_available())
    {
        // create a tensor and send it to the GPU
        auto cuda_tensor = torch::tensor({1.0, 2.0, 3.0}).to("cuda");
    }

    // create a tensor of shape 2x4 filled with ones
    std::cout << torch::ones({2, 4}) << std::endl;

    return 0;
}
#else
#include <iostream>
int main()
{

    std::cout << "This example requires PyTorch and gymfcpp. Reconfigure bitrl with USE_PYTORCH "
                 "flags turned ON."
              << std::endl;
    return 0;
}
#endif
