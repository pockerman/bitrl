\page cuberl_example_8 CubeRL Example 8: Working with CUDA

Deep learning methods often require excesive training times due to the amount of data that they have to process.
One way to speed up such applications is to move aspects of the computation into GPU devices. _CubeRL_ can be configured
to use any available GPU devices by utilising the <a href="https://developer.nvidia.com/cuda-toolkit">CUDA</a> framework.
This of course means that you need to have a CUDA capable device.

In this example we will set up as small vector addition operation  in order to show you how to use CUDA and how to
transfer computation on a GPU device. In general, programming GPUs is not necessarilly easy and it involves a lot
of boilerplate code. Frameworks such as <a href="https://github.com/kokkos/kokkos">Kokkos</a> and  
<a href="https://github.com/LLNL/RAJA">RAJA</a> attempt to provide functionality for prompting the performance we gain from the GPU
to various platforms as seamlessly as possible.


Note that this example requires that you have configured _CubeRL_ with ```USE_CUDA=ON```.
Note also that the integration with CUDA is highly experimental at the moment of writing this example.

## The driver code

The driver code for this tutorial is shown below.

```cpp
#include <stdio.h>
#include <cuda.h>

#include "cubeai/base/cubeai_config.h"

#ifdef USE_CUDA

#include "cubeai/base/cubeai_types.h"

#include <boost/log/trivial.hpp>
#include <iostream>

namespace exe{
	
	using cubeai::float_t;

__global__ void sum(float_t* v1, float_t* v2, float_t* v3){

	// the thread id we use to correctly
	// access the vector prosition we
	// are interested in
	int idx = threadIdx.x;
	float_t f1 = v1[idx];
	float_t f2 = v2[idx];
	float_t f3 = f1 + f2;
	v3[idx] = f3;

}

}

int main() {
	
	using namespace exe;
	
	BOOST_LOG_TRIVIAL(info)<<"Running example...";
    
	const int ARRAY_SIZE = 64;
	const int ARRAY_BYTES = ARRAY_SIZE * sizeof(float_t);

	// host arrays
	float_t h_v1[ARRAY_SIZE];
	float_t h_v2[ARRAY_SIZE];
	float_t h_v3[ARRAY_SIZE];
	
	for(int i=0; i<ARRAY_SIZE; ++i){
	   h_v1[i] = float_t(i);
	   h_v2[i] = float_t(i);
	   h_v3[i] = 0.0f;
	}

	// device arrays
	float_t* d_v1 = nullptr;
	float_t* d_v2 = nullptr;
	float_t* d_v3 = nullptr;

	// allocate GPU memory for the device arrays
	cudaMalloc((void **) &d_v1, ARRAY_BYTES);
	cudaMalloc((void **) &d_v2, ARRAY_BYTES);
	cudaMalloc((void **) &d_v3, ARRAY_BYTES);

	// transfer array to GPU 
	cudaMemcpy(d_v1, h_v1, ARRAY_BYTES, cudaMemcpyHostToDevice);
	cudaMemcpy(d_v2, h_v2, ARRAY_BYTES, cudaMemcpyHostToDevice);
	cudaMemcpy(d_v3, h_v3, ARRAY_BYTES, cudaMemcpyHostToDevice);

	// launch kernel
	sum<<<1, ARRAY_SIZE>>>(d_v1, d_v2, d_v3);

	// copy the output from the GPU to host
	cudaMemcpy(h_v3, d_v3, ARRAY_BYTES, cudaMemcpyDeviceToHost);


	for(int i=0; i<ARRAY_SIZE; ++i){
		std::cout<<h_v1[i]<<"+"<<h_v2[i]<<"="<<h_v3[i]<<std::endl;
	}

	// free memory
	cudaFree(d_v1);
	cudaFree(d_v2);
	cudaFree(d_v3);
		
    cudaDeviceReset();
	
	BOOST_LOG_TRIVIAL(info)<<"Done...";
    return 0;
}

#else
#include <iostream>
int main() {
	std::cout<<"This example requires CUDA support enabled. Reconfigure CubeRL and set USE_CUDA=ON"<<std::endl;
	return 1;
}
#endif

```

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

