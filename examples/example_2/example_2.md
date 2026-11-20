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


\include example_2/example_2.cpp
