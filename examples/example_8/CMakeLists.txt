ADD_EXECUTABLE(example_8 example_8.cu)

# Build all CUDA files with -dc as the member functions
# could be called by other libraries and executables
SET_TARGET_PROPERTIES(example_8 PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
TARGET_LINK_LIBRARIES(example_8 PRIVATE cuberllib)
