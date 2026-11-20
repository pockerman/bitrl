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


\include example_3/example_3.cpp

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

