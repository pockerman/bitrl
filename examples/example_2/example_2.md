\page bitrl_example_2 bitrl Example 2: Using PyTorch C++ API Part 1

Example intro_example_1 demonstrated some basic functionality that _bitrl_ implements.
In addition, _bitrl_ is heavily based on the C++ API exposed by PyTorch. 
Therefore, this tutorial is meant to show you some basics on how to create and manipulate PyTorch
tensors.  There are many resources you can consult for further info:

- <a href="https://pytorch.org/cppdocs/">PyTorch C++ API</a>
- <a href="https://pytorch.org/tutorials/advanced/cpp_frontend.html">Using the PyTorch C++ Frontend</a>
- <a href="https://github.com/prabhuomkar/pytorch-cpp/tree/master">pytorch-cpp</a>

Note that this tutorial requires bitrl to be configured with PyTorch support.


## The driver code

The driver code for this tutorial is shown below. 


@code
#include "cubeai/base/cubeai_config.h"

#if defined(USE_PYTORCH)

#include "cubeai/base/cubeai_types.h"


#include <torch/torch.h>
#include <boost/log/trivial.hpp>

#include <string>
#include <vector>
#include <iostream>

namespace example2{
	
	using cubeai::real_t;
	
	// this type is not supported by PyTorch
	using cubeai::uint_t;
	using cubeai::int_t;
}


int main() {

  using namespace example2;

  if(torch::cuda::is_available()){
  	std::cout<<"CUDA is available on this machine"<<std::endl;
  }
  else{
  	std::cout<<"CUDA is not available on this machine"<<std::endl;
  }

  // various methods to create a tensor
  torch::Tensor tensor = torch::eye(3);
  std::cout << tensor << std::endl;
  
  // floating point tensor
  std::vector<real_t> data(3, 2.0);
  auto tensor_from_data_1 = torch::tensor(data);
  std::cout << tensor_from_data_1 << std::endl; 
  
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

  if(torch::cuda::is_available()){
	   // create a tensor and send it to the GPU
	   auto cuda_tensor = torch::tensor({1.0, 2.0, 3.0}).to("cuda");
   
  }
  
  // create a tensor of shape 2x4 filled with ones
  std::cout << torch::ones({2, 4})<< std::endl;
  
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
