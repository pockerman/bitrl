INCLUDE(GoogleTest)

# Each test links cuberllib and GTest; bitrl, libtorch and TBB come with cuberllib
FUNCTION(cuberl_add_test name)
	ADD_EXECUTABLE(${name} ${name}.cpp)
	TARGET_LINK_LIBRARIES(${name} PRIVATE cuberllib GTest::gtest GTest::gtest_main)
	cuberl_set_warnings(${name})
	GTEST_DISCOVER_TESTS(${name} DISCOVERY_TIMEOUT 60)
ENDFUNCTION()

ADD_SUBDIRECTORY(test_policies/test_max_tabular_policy)
ADD_SUBDIRECTORY(test_policies/test_random_tabular_policy)
ADD_SUBDIRECTORY(test_policies/test_epsilon_greedy_policy)
ADD_SUBDIRECTORY(test_policies/test_softmax_policy)
ADD_SUBDIRECTORY(test_maths/test_vector_math)
ADD_SUBDIRECTORY(test_experience_buffer)
ADD_SUBDIRECTORY(test_grid_world)
ADD_SUBDIRECTORY(test_connect2)
