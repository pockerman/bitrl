\page bitrl_example_cuda_vector_addition Vector addition with CUDA

Deep learning methods often require excesive training times due to the amount of data that they have to process.
One way to speed up such applications is to move aspects of the computation into GPU devices. _bitrl_ can be configured
to use any available GPU devices by utilising the <a href="https://developer.nvidia.com/cuda-toolkit">CUDA</a> framework.
This of course means that you need to have a CUDA capable device.

In this example we will set up as small vector addition operation  in order to show you how to use CUDA and how to
transfer computation on a GPU device. In general, programming GPUs is not necessarilly easy and it involves a lot
of boilerplate code. Frameworks such as <a href="https://github.com/kokkos/kokkos">Kokkos</a> and  
<a href="https://github.com/LLNL/RAJA">RAJA</a> attempt to provide functionality for prompting the performance we gain from the GPU
to various platforms as seamlessly as possible.


Note that this example requires that you have configured _bitrl_ with ```USE_CUDA=ON```.
Note also that the integration with CUDA is highly experimental at the moment of writing this example.

## The driver code

The driver code for this tutorial is shown below.

\include cuda_vector_addition/cuda_vector_addition.cu

Running the code above produces the following:

```
Running example...
0+0=0
1+1=2
2+2=4
3+3=6
4+4=8
5+5=10
6+6=12
7+7=14
8+8=16
9+9=18
10+10=20
11+11=22
12+12=24
13+13=26
14+14=28
15+15=30
16+16=32
17+17=34
18+18=36
19+19=38
...
Done...

```

