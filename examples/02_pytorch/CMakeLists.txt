# The PyTorch C++ API that bitrl's deep reinforcement learning is built on
bitrl_add_example(pytorch_tensors FILES config.json SMOKE_TEST)
bitrl_add_example(pytorch_linear_regression FILES config.json SMOKE_TEST)
bitrl_add_example(pytorch_custom_model FILES config.json)

# needs CUDA, and libtorch built with it
IF(USE_CUDA)
	ADD_EXECUTABLE(cuda_vector_addition cuda_vector_addition/cuda_vector_addition.cu)
	# build the CUDA files with -dc, as their functions may be called from
	# other libraries and executables
	SET_TARGET_PROPERTIES(cuda_vector_addition PROPERTIES CUDA_SEPARABLE_COMPILATION ON
		RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/cuda_vector_addition)
	TARGET_LINK_LIBRARIES(cuda_vector_addition PRIVATE bitrllib)
ENDIF()
