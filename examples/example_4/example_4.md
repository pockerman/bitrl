\page bitrl_example_4 bitrl Example 4: Using PyTorch C++ API Part 3

Example 3 showed you how to create a simple PyTorch model for linear regression.
However, we did so by utilzing _torch::nn::Linear_. In this example,
we want to go a step further and see how to build our own models.
If you have used PyTorch with Python you will see that the process is
not very different. We will also see how to load a dataset and save
a PyTorch model such that we can then load in Python.


Specifically, we will implement a simple logistic regression model. We will use the
MNIST dataset. Make sure that you have downloaded the dataset and the arrangement has the
following directory structure. It may also be useful to have a look at the source
code for the PyTorch MNIST class here: https://github.com/pytorch/pytorch/blob/main/torch/csrc/api/src/data/datasets/mnist.cpp

- train images: train-images-idx3-ubyte
- train labels: train-labels-idx1-ubyte
- test images: t10k-images-idx3-ubyte
- test labels: t10k-labels-idx1-ubyte

You can download the data from either here: http://yann.lecun.com/exdb/mnist/ or here: https://www.kaggle.com/datasets/hojjatk/mnist-dataset

## The driver code

The driver code for this tutorial is shown below.

```cpp
#include "cubeai/base/cubeai_config.h"

#if defined(USE_PYTORCH)

#include "cubeai/base/cubeai_types.h"
#include "cubeai/utils/iteration_counter.h"
#include "cubeai/utils/cubeai_concepts.h"
#include "cubeai/geom_primitives/shapes/circle.h"
#include "cubeai/io/json_file_reader.h"


#include "cubeai/base/cubeai_data_paths.h"
#include "cubeai/optimization/optimizer_type.h"
#include "cubeai/ml/loss_type.h"
#include "cubeai/ml/pytorch_supervised_trainer.h"
#include "cubeai/ml/pytorch_loss_wrapper.h"

#include <torch/torch.h>
#include <boost/log/trivial.hpp>

#include <memory>
#include <iostream>
#include <random>
#include <string>

namespace intro_example_3
{

using cubeai::real_t;
using cubeai::uint_t;
using cubeai::torch_tensor_t;
using cubeai::io::JSONFileReader;

namespace fs = std::filesystem;
const std::string CONFIG = "config.json";


// The regression model
class LogisticRegressionModelImpl: public torch::nn::Module
{

public:

    // constructor. Construct the model by passing
    // the number of features and the number of classes
    LogisticRegressionModelImpl(uint_t input_size, uint_t output_size);

    // forward
    torch_tensor_t forward(torch_tensor_t input);

    // test the model
    void test_model(const std::string& test_path, uint_t batch_size,
              torch::Device device);

    // train the model
    void train_model(const std::string& train_path, uint_t batch_size,
                     uint_t n_epochs, real_t lr, torch::Device device);

private:

    torch::nn::Linear linear_;
    torch_tensor_t bias_;

};


LogisticRegressionModelImpl::LogisticRegressionModelImpl(uint_t input_size, uint_t output_size)
:
linear_(nullptr)
{
    linear_ = register_module("linear_", torch::nn::Linear(input_size, output_size));
    bias_ = register_parameter("bias_", torch::randn(output_size));
}

torch_tensor_t
LogisticRegressionModelImpl::forward(torch_tensor_t input){

  return torch::sigmoid(linear_(input) + bias_);
}

void
LogisticRegressionModelImpl::test_model(const std::string& test_set_path,
                                        uint_t batch_size, torch::Device device){

  BOOST_LOG_TRIVIAL(info)<<"Testing starts..."<<std::endl;;

  // Test the model
  this->eval();
  torch::NoGradGuard no_grad;

  auto test_dataset = torch::data::datasets::MNIST(test_set_path,
                                                   torch::data::datasets::MNIST::Mode::kTest)
              .map(torch::data::transforms::Normalize<>(0.1307, 0.3081))
              .map(torch::data::transforms::Stack<>());

  // Number of samples in the testset
  auto num_test_samples = test_dataset.size().value();
  BOOST_LOG_TRIVIAL(info)<<"Number of test examples "<<num_test_samples<<std::endl;

  auto test_loader = torch::data::make_data_loader<torch::data::samplers::SequentialSampler>(std::move(test_dataset),
                                                                                             batch_size);

  real_t running_loss = 0.0;
  uint_t num_correct = 0;

  for (const auto& batch : *test_loader) {

      auto data = batch.data.view({batch_size, -1}).to(device);
      auto target = batch.target.to(device);

      auto output = this->forward(data);

      auto loss = torch::nn::functional::cross_entropy(output, target);
      running_loss += loss.item<real_t>() * data.size(0);

      auto prediction = output.argmax(1);
      num_correct += prediction.eq(target).sum().item<int64_t>();
  }


  BOOST_LOG_TRIVIAL(info)<<"Testing finished..."<<std::endl;

  auto test_accuracy = static_cast<double>(num_correct) / num_test_samples;
  auto test_sample_mean_loss = running_loss / num_test_samples;
  BOOST_LOG_TRIVIAL(info)<<"Testset - Loss: "<< test_sample_mean_loss << ", Accuracy: "<< test_accuracy;

}


void
LogisticRegressionModelImpl::train_model(const std::string& train_set_path,
                                         uint_t batch_size, uint_t n_epochs,
                                         real_t lr, torch::Device device){

  BOOST_LOG_TRIVIAL(info)<<"Training starts..."<<std::endl;

  auto train_dataset = torch::data::datasets::MNIST(train_set_path)
                                                    //torch::data::datasets::MNIST::Mode::kTrain)
             .map(torch::data::transforms::Normalize<>(0.1307, 0.3081))
             .map(torch::data::transforms::Stack<>());

  // Number of samples in the training set
  auto num_train_samples = train_dataset.size().value();
  BOOST_LOG_TRIVIAL(info)<<"Number of train examples "<<num_train_samples;

  auto train_loader = torch::data::make_data_loader<torch::data::samplers::RandomSampler>(std::move(train_dataset),
                                                                                          batch_size);

  auto optimizer = std::make_unique<torch::optim::SGD>(this->parameters(), torch::optim::SGDOptions{lr});


  // Train the model
  for (size_t epoch = 0; epoch != n_epochs; ++epoch) {

      // Initialize running metrics
      auto running_loss = 0.0;
      uint_t num_correct = 0;

      for (auto& batch : *train_loader) {

          auto data = batch.data.view({static_cast<long int>(batch_size), -1}).to(device);
          auto target = batch.target.to(device);

          // Forward pass
          auto output = this->forward(data);

          // Calculate loss. Use cross_entropy
          auto loss = torch::nn::functional::cross_entropy(output, target);

          // Update running loss
          running_loss += loss. template item<real_t>() * data.size(0);

          // Calculate prediction
          auto prediction = output.argmax(1);

          // Update number of correctly classified samples
          num_correct += prediction.eq(target).sum(). template item<int64_t>();

          // Backward pass and optimize
          optimizer->zero_grad();
          loss.backward();
          optimizer->step();
        }

        auto sample_mean_loss = running_loss / num_train_samples;
        auto accuracy = static_cast<real_t>(num_correct) / num_train_samples;

        BOOST_LOG_TRIVIAL(info)<< "Epoch [" << (epoch + 1) << "/" << n_epochs << "], Trainset - Loss: "
                              << sample_mean_loss << ", Accuracy: " << accuracy << '\n';
    }

    BOOST_LOG_TRIVIAL(info)<<"Training finished..."<<std::endl;

}

TORCH_MODULE(LogisticRegressionModel);

}

int main() {

    using namespace intro_example_3;

    try{

      // load the json configuration
      JSONFileReader json_reader(CONFIG);
      json_reader.open();

      // figure out the device we are using
      auto cuda_available = torch::cuda::is_available();
      torch::Device device(cuda_available ? torch::kCUDA : torch::kCPU);
      if(cuda_available){
        BOOST_LOG_TRIVIAL(info)<<"CUDA available. Training on GPU "<<std::endl;
      }
      else{
        BOOST_LOG_TRIVIAL(info)<<"CUDA is not available. Training on CPU "<<std::endl;
      }

      auto train_set_data_path = json_reader.template get_value<std::string>("train_data_path");
      auto test_set_data_path = json_reader.template get_value<std::string>("test_data_path");
      auto seed = json_reader.template get_value<uint_t>("seed");
      auto input_size = json_reader.template get_value<uint_t>("input_size");
      auto num_classes = json_reader.template get_value<uint_t>("num_classes");
      auto batch_size = json_reader.template get_value<uint_t>("batch_size");
      auto num_epochs = json_reader.template get_value<uint_t>("num_epochs");
      auto lr = json_reader.template get_value<real_t>("lr");

      // set the seed
      torch::manual_seed(seed);

      LogisticRegressionModel model(input_size, num_classes);

      model -> to(device);
      model -> train_model(train_set_data_path, batch_size, num_epochs, lr,device);
      //model -> test_model(test_set_data_path, batch_size, device);

    }
    catch(std::exception& e){
        std::cout<<e.what()<<std::endl;
    }
    catch(...){

        std::cout<<"Unknown exception occured"<<std::endl;
    }

   return 0;
}
#else
#include <iostream>
int main(){

    std::cout<<"This example requires PyTorch. Reconfigure bitrl with USE_PYTORCH  flag turned ON."<<std::endl;
    return 0;
}
#endif

```
Running the code above produces the following output
