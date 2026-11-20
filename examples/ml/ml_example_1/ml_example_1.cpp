/**
 * Example 21: Implements a simple logistic regression
 * model with PyTorch. The example is editted from this
 * https://github.com/prabhuomkar/pytorch-cpp/blob/master/tutorials/basics/logistic_regression/main.cpp
 * GitHub repository. The example uses the MNIST dataset
 * The train and test files should be respectively
 *
 * train images: train-images-idx3-ubyte
 * train labels: train-labels-idx1-ubyte
 *
 * test images: t10k-images-idx3-ubyte
 * test labels: t10k-labels-idx1-ubyte
 *
 */

#include "cubeai/base/cubeai_config.h"

#ifdef USE_PYTORCH

#include "cubeai/base/cubeai_data_paths.h"
#include "cubeai/base/cubeai_types.h"
#include "cubeai/ml/loss_type.h"
#include "cubeai/ml/pytorch_loss_wrapper.h"
#include "cubeai/ml/pytorch_supervised_trainer.h"
#include "cubeai/optimization/optimizer_type.h"

#include <iostream>
#include <torch/torch.h>

namespace example21
{

using cubeai::real_t;
using cubeai::torch_tensor_t;
using cubeai::uint_t;
using cubeai::ml::LossType;
using cubeai::ml::pytorch::PyTorchLossWrapper;
using cubeai::ml::pytorch::PyTorchSupervisedTrainer;
using cubeai::ml::pytorch::PyTorchSupervisedTrainerConfig;
using cubeai::optim::OptimzerType;

const uint_t input_size = 784;
const uint_t num_classes = 10;
const uint_t batch_size = 100;
const uint_t num_epochs = 5;
const real_t learning_rate = 0.001;

const std::string MNIST_train_data_path =
    cubeai::CubeAIDataPaths::mnist_data_train_directory_path();
const std::string MNIST_test_data_path = cubeai::CubeAIDataPaths::mnist_data_test_directory_path();

// The regression model
class LogisticRegressionModel : public torch::nn::Module
{

  public:
    // constructor. Construct the model by passing
    // the number of features and the number of classes
    LogisticRegressionModel(uint_t input_size, uint_t output_size);

    // forward
    torch_tensor_t forward(torch_tensor_t input);

  private:
    torch::nn::Linear linear;
    torch_tensor_t bias_;
};

LogisticRegressionModel::LogisticRegressionModel(uint_t input_size, uint_t output_size)
    : linear(register_module("linear", torch::nn::Linear(input_size, output_size))), bias_()
{
    bias_ = register_parameter("b", torch::randn(output_size));
}

torch_tensor_t LogisticRegressionModel::forward(torch_tensor_t input)
{
    return torch::sigmoid(linear(input) + bias_);
}

} // namespace example21

int main()
{

    using namespace example21;

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

        // Device
        auto cuda_available = torch::cuda::is_available();
        torch::Device device(cuda_available ? torch::kCUDA : torch::kCPU);

        // load the data
        // MNIST Dataset (images and labels)
        auto train_dataset = torch::data::datasets::MNIST(
                                 MNIST_train_data_path, torch::data::datasets::MNIST::Mode::kTrain)
                                 .map(torch::data::transforms::Normalize<>(0.1307, 0.3081))
                                 .map(torch::data::transforms::Stack<>());

        // Number of samples in the training set
        auto num_train_samples = train_dataset.size().value();
        std::cout << cubeai::CubeAIConsts::info_str() << "Number of train examples "
                  << num_train_samples << std::endl;

        PyTorchSupervisedTrainerConfig config;
        config.n_epochs = num_epochs;
        config.batch_size = batch_size;
        config.device = device;
        config.optim_type = OptimzerType::SGD;

        std::map<std::string, std::any> optim_ops;
        optim_ops["lr"] = std::any(learning_rate);
        config.optim_options = optim_ops;

        PyTorchLossWrapper loss(LossType::CROSS_ENTROPY);

        LogisticRegressionModel model(input_size, num_classes);
        model.to(device);

        PyTorchSupervisedTrainer<LogisticRegressionModel> trainer(config, model);
        trainer.train(train_dataset, loss);

        std::cout << cubeai::CubeAIConsts::info_str() << "Training finished..." << std::endl;
        std::cout << cubeai::CubeAIConsts::info_str() << "Start testing..." << std::endl;

        auto test_dataset = torch::data::datasets::MNIST(MNIST_test_data_path,
                                                         torch::data::datasets::MNIST::Mode::kTest)
                                .map(torch::data::transforms::Normalize<>(0.1307, 0.3081))
                                .map(torch::data::transforms::Stack<>());

        // Number of samples in the test set
        auto num_test_samples = test_dataset.size().value();
        std::cout << cubeai::CubeAIConsts::info_str() << "Number of test examples "
                  << num_test_samples << std::endl;

        auto test_loader = torch::data::make_data_loader<torch::data::samplers::SequentialSampler>(
            std::move(test_dataset), batch_size);

        // Test the model
        model.eval();
        torch::NoGradGuard no_grad;

        real_t running_loss = 0.0;
        uint_t num_correct = 0;

        for (const auto &batch : *test_loader)
        {

            auto data = batch.data.view({batch_size, -1}).to(device);
            auto target = batch.target.to(device);

            auto output = model.forward(data);

            auto loss = torch::nn::functional::cross_entropy(output, target);

            running_loss += loss.item<real_t>() * data.size(0);

            auto prediction = output.argmax(1);

            num_correct += prediction.eq(target).sum().item<int64_t>();
        }

        std::cout << cubeai::CubeAIConsts::info_str() << "Testing finished..." << std::endl;
        ;

        auto test_accuracy = static_cast<double>(num_correct) / num_test_samples;
        auto test_sample_mean_loss = running_loss / num_test_samples;

        std::cout << cubeai::CubeAIConsts::info_str() << "Testset - Loss: " << test_sample_mean_loss
                  << ", Accuracy: " << test_accuracy << std::endl;
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
