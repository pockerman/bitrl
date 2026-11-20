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

\include example_4/example_4.cpp
Running the code above produces the following output
