#include "cubeai/base/cubeai_config.h"

#ifdef USE_PYTORCH

#include "cubeai/base/cubeai_types.h"

#include <iostream>
#include <torch/torch.h>

namespace example
{

using cubeai::real_t;
using cubeai::torch_tensor_t;
using cubeai::uint_t;

// The regression model
class LinearRegressionModel : public torch::nn::Module
{

  public:
    //
    LinearRegressionModel(uint_t input_size, uint_t output_size);

    // forward
    torch_tensor_t forward(torch_tensor_t input);

  private:
    torch::nn::Linear linear;
    torch_tensor_t bias_;
};

LinearRegressionModel::LinearRegressionModel(uint_t input_size, uint_t output_size)
    : linear(register_module("linear", torch::nn::Linear(input_size, output_size))), bias_()
{
    bias_ = register_parameter("b", torch::randn(output_size));
}

torch_tensor_t LinearRegressionModel::forward(torch_tensor_t input)
{
    return linear(input) + bias_;
}

} // namespace example

int main()
{

    using namespace example;

    try
    {

        if (torch::cuda::is_available())
        {
            std::cout << "CUDA is available on this machine" << std::endl;
        }
        else
        {
            std::cout << "CUDA is not available on this machine" << std::endl;
        }

        // create data
        std::vector<double> x_train(11, 0.0);
        std::vector<double> y_train(11, 0.0);

        for (uint_t i = 0; i < x_train.size(); ++i)
        {
            x_train[i] = static_cast<double>(i);
            y_train[i] = 2 * static_cast<double>(i) + 1;
        }

        auto x_tensor = torch::from_blob(
            x_train.data(), {int(y_train.size()), int(x_train.size() / y_train.size())});
        auto y = torch::from_blob(y_train.data(), {int(y_train.size()), 1});

        LinearRegressionModel net(1, 1);
        for (const auto &p : net.parameters())
        {
            std::cout << p << std::endl;
        }

        torch::nn::MSELoss mse;
        torch::optim::SGD sgd(net.parameters(), 0.01);

        for (uint_t e = 0; e < 100; ++e)
        {

            sgd.zero_grad();

            auto outputs = net.forward(x_tensor);
            auto loss = mse(outputs, y);

            // get gradients w.r.t to parameters
            loss.backward();

            // update parameters
            sgd.step();
            std::cout << "Epoch=" << e << " loss=" << loss << std::endl;
        }
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    catch (...)
    {

        std::cout << "Unknown exception occured" << std::endl;
    }
    return 0;
}
#else
#include <iostream>
int main()
{

    std::cout << "This example requires PyTorch. Reconfigure cubeai with PyTorch support."
              << std::endl;
    return 0;
}
#endif
