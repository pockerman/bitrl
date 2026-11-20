\page bitrl_example_3 bitrl Example 3: Using PyTorch C++ API Part 2

Example intro_example_1 demonstrated some functionality
bitrl is heavily based on the C++ API exposed by PyTorch. 
Therefore, this tutorial is mean to show you some basics on how to create
and train PyTorch models using the C++ API. There are many resources you can
consult for further info:

- <a href="https://pytorch.org/cppdocs/">PyTorch C++ API</a>
- <a href="https://pytorch.org/tutorials/advanced/cpp_frontend.html">Using the PyTorch C++ Frontend</a>
- <a href="https://github.com/prabhuomkar/pytorch-cpp/tree/master">pytorch-cpp</a>

In this tutorial we will reproduce the code in <a href="https://github.com/prabhuomkar/pytorch-cpp/blob/master/tutorials/basics/linear_regression/main.cpp">linear regression with PyTorch</a>.
We will also show how to read a json file with bitrl, how to use the ```IterativeAlgorithmController``` class. Finally, we will see how to use the logging utilities from the boost C++ library.
Note that this tutorial requires bitrl to be configured with PyTorch support.


## The driver code

The driver code for this tutorial is shown below. 


@code
#include "cubeai/base/cubeai_config.h"

#ifdef USE_PYTORCH

#include "cubeai/base/cubeai_types.h"
#include "bitsim/utils/iteration_counter.h"
#include "bitsim/utils/io/json_file_reader.h"

#include <torch/torch.h>
#include <boost/log/trivial.hpp>

#include <random>
#include <filesystem>
#include <string>

namespace example
{

using bitrl::real_t;
using bitrl::uint_t;
using bitsim::utils::io::JSONFileReader;
using bitsim::utils::IterationCounter;

namespace fs = std::filesystem;
const std::string CONFIG = "config.json";


std::string train_model()
{
try{

      // load the json configuration
      JSONFileReader json_reader(CONFIG);
      json_reader.open();

      auto experiment_dict = json_reader.template get_value<std::string>("experiment_dict");
      auto experiment_id = json_reader.template get_value<std::string>("experiment_id");

      BOOST_LOG_TRIVIAL(info)<<"Experiment directory: "<<experiment_dict;
      BOOST_LOG_TRIVIAL(info)<<"Experiment id: "<<experiment_id<<std::endl;

      const fs::path EXPERIMENT_DIR_PATH = experiment_dict + experiment_id;

      // the first thing we want to do when monitoring experiments
      // is to create a directory where all data will reside
      std::filesystem::create_directories(experiment_dict + experiment_id);

      const auto input_size = json_reader.template get_value<uint_t>("input_size");
      const auto output_size = json_reader.template get_value<uint_t>("output_size");
      const auto num_epochs = json_reader.template get_value<uint_t>("num_epochs");
      const auto learning_rate = json_reader.template get_value<real_t>("lr");

      // log the hyperparameters
      BOOST_LOG_TRIVIAL(info)<<"Input size: "<<input_size;
      BOOST_LOG_TRIVIAL(info)<<"Output size: "<<output_size;
      BOOST_LOG_TRIVIAL(info)<<"Max epochs: "<<num_epochs;
      BOOST_LOG_TRIVIAL(info)<<"Learning rate: "<<learning_rate;

      // figure out the device we are using
      auto cuda_available = torch::cuda::is_available();
      torch::Device device(cuda_available ? torch::kCUDA : torch::kCPU);
      if(cuda_available){
        BOOST_LOG_TRIVIAL(info)<<"CUDA available. Training on GPU "<<std::endl;
      }
      else{
        BOOST_LOG_TRIVIAL(info)<<"CUDA is not available. Training on CPU "<<std::endl;
      }

      // Sample dataset
      auto x_train = torch::randint(0, 10, {15, 1},
                                    torch::TensorOptions(torch::kFloat).device(device));

      auto y_train = torch::randint(0, 10, {15, 1},
                                    torch::TensorOptions(torch::kFloat).device(device));

      // Linear regression model
      torch::nn::Linear model(input_size, output_size);
      model->to(device);

      // Optimizer
      torch::optim::SGD optimizer(model->parameters(),
                                  torch::optim::SGDOptions(learning_rate));

      BOOST_LOG_TRIVIAL(info)<<"Start training...";

      IterationCounter iteration_ctrl(num_epochs);
      while(iteration_ctrl.continue_iterations()){

          // Forward pass
          auto output = model->forward(x_train);
          auto loss = torch::nn::functional::mse_loss(output, y_train);

          // Backward pass and optimize
          optimizer.zero_grad();
          loss.backward();
          optimizer.step();

          auto current_iteration = iteration_ctrl.current_iteration_index();
          if ((current_iteration + 1) % 5 == 0) {
            BOOST_LOG_TRIVIAL(info)<< "Epoch [" << (current_iteration + 1) << "/" << num_epochs <<"], Loss: " << loss.item<double>(); //<< "\n";
          }
      }

      BOOST_LOG_TRIVIAL(info)<<"Training finished... ";

      auto model_filename = std::string(EXPERIMENT_DIR_PATH) + std::string("/linear_regression_model.pth");
      BOOST_LOG_TRIVIAL(info)<<"Saving model at: "<<model_filename<<std::endl;

      torch::save(model, model_filename);
      return model_filename;
    }
    catch(std::exception& e){
        std::cout<<e.what()<<std::endl;
    }
    catch(...){
        std::cout<<"Unknown exception occurred"<<std::endl;
    }
}

void test_model(std::string& model_filename)
{
BOOST_LOG_TRIVIAL(info)<<"Start testing...";
// load the json configuration
JSONFileReader json_reader(CONFIG);
json_reader.open();

const auto input_size = json_reader.template get_value<uint_t>("input_size");
const auto output_size = json_reader.template get_value<uint_t>("output_size");

// figure out the device we are using
auto cuda_available = torch::cuda::is_available();
torch::Device device(cuda_available ? torch::kCUDA : torch::kCPU);
if(cuda_available){
BOOST_LOG_TRIVIAL(info)<<"CUDA available. Testing on GPU ";
}
else{
BOOST_LOG_TRIVIAL(info)<<"CUDA is not available. Testing on CPU ";
}

torch::nn::Linear model(input_size, output_size);
torch::load(model, model_filename);
model->to(device);

// we will evaluate the model
model -> eval();

// Sample dataset
auto x_test = torch::randint(0, 10, {15, 1},
torch::TensorOptions(torch::kFloat).device(device));
// get the test predictions
auto output = model->forward(x_test);

// do some comparisons
// ...

BOOST_LOG_TRIVIAL(info)<<"Testing finished... ";
}

}

int main() {

    using namespace example;

    // set seed for torch
    torch::manual_seed(42);
    auto model_filename = train_model();

    test_model(model_filename);

return 0;
}
#else
#include <iostream>
int main(){

    std::cout<<"This example requires PyTorch and gymfcpp. Reconfigure bitrl with USE_PYTORCH flags turned ON."<<std::endl;
    return 0;
}
#endif
@endcode

Running the code above produces the following output

```
[2026-01-10 10:52:16.766804] [0x00007f3fc6bdc6c0] [info]    Experiment directory: intro_example_2_experiments/
[2026-01-10 10:52:16.766822] [0x00007f3fc6bdc6c0] [info]    Experiment id: 1

[2026-01-10 10:52:16.766851] [0x00007f3fc6bdc6c0] [info]    Input size: 1
[2026-01-10 10:52:16.766855] [0x00007f3fc6bdc6c0] [info]    Output size: 1
[2026-01-10 10:52:16.766858] [0x00007f3fc6bdc6c0] [info]    Max epochs: 60
[2026-01-10 10:52:16.766873] [0x00007f3fc6bdc6c0] [info]    Learning rate: 0.001
[2026-01-10 10:52:16.766880] [0x00007f3fc6bdc6c0] [info]    CUDA is not available. Training on CPU 

[2026-01-10 10:52:16.785356] [0x00007f3fc6bdc6c0] [info]    Start training...
[2026-01-10 10:52:16.809743] [0x00007f3fc6bdc6c0] [info]    Epoch [5/60], Loss: 23.7257
[2026-01-10 10:52:16.810012] [0x00007f3fc6bdc6c0] [info]    Epoch [10/60], Loss: 23.3023
[2026-01-10 10:52:16.810254] [0x00007f3fc6bdc6c0] [info]    Epoch [15/60], Loss: 22.9896
[2026-01-10 10:52:16.810584] [0x00007f3fc6bdc6c0] [info]    Epoch [20/60], Loss: 22.751
[2026-01-10 10:52:16.810948] [0x00007f3fc6bdc6c0] [info]    Epoch [25/60], Loss: 22.5618
[2026-01-10 10:52:16.811305] [0x00007f3fc6bdc6c0] [info]    Epoch [30/60], Loss: 22.4059
[2026-01-10 10:52:16.811655] [0x00007f3fc6bdc6c0] [info]    Epoch [35/60], Loss: 22.2722
[2026-01-10 10:52:16.811899] [0x00007f3fc6bdc6c0] [info]    Epoch [40/60], Loss: 22.1536
[2026-01-10 10:52:16.812296] [0x00007f3fc6bdc6c0] [info]    Epoch [45/60], Loss: 22.0452
[2026-01-10 10:52:16.812528] [0x00007f3fc6bdc6c0] [info]    Epoch [50/60], Loss: 21.9438
[2026-01-10 10:52:16.812779] [0x00007f3fc6bdc6c0] [info]    Epoch [55/60], Loss: 21.8473
[2026-01-10 10:52:16.813002] [0x00007f3fc6bdc6c0] [info]    Epoch [60/60], Loss: 21.7542
[2026-01-10 10:52:16.813055] [0x00007f3fc6bdc6c0] [info]    Training finished... 
[2026-01-10 10:52:16.813060] [0x00007f3fc6bdc6c0] [info]    Saving model at: intro_example_2_experiments/1/linear_regression_model.pth

[2026-01-10 10:52:16.815886] [0x00007f3fc6bdc6c0] [info]    Start testing...
[2026-01-10 10:52:16.815932] [0x00007f3fc6bdc6c0] [info]    CUDA is not available. Testing on CPU 
[2026-01-10 10:52:16.828503] [0x00007f3fc6bdc6c0] [info]    Testing finished... 

```

