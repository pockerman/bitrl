INCLUDE(GoogleTest)

# Each test links bitrllib and GTest; bitsim, libtorch and TBB come with bitrllib
FUNCTION(bitrl_add_test name)
	ADD_EXECUTABLE(${name} ${name}.cpp)
	TARGET_LINK_LIBRARIES(${name} PRIVATE bitrllib GTest::gtest GTest::gtest_main)
	bitrl_set_warnings(${name})
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
