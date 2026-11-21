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
	std::cout<<"This example requires CUDA support enabled. Reconfigure bitrl and set USE_CUDA=ON"<<std::endl;
	return 1;
}
#endif