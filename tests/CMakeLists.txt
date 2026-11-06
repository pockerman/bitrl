CMAKE_MINIMUM_REQUIRED(VERSION 3.20)

# Adds the test <name> built from <name>/<name>.cpp and registers it with CTest.
FUNCTION(BITRL_ADD_TEST name)
	ADD_EXECUTABLE(${name} ${name}/${name}.cpp)
	TARGET_LINK_LIBRARIES(${name} PRIVATE bitrllib GTest::gtest GTest::gtest_main)
	ADD_TEST(NAME ${name} COMMAND ${name})
ENDFUNCTION()

# Tests that run on their own
BITRL_ADD_TEST(test_grid_world)
BITRL_ADD_TEST(test_generic_line)

IF(BITRL_CHRONO)
	BITRL_ADD_TEST(test_chrono_ultrasound)
ENDIF()

# TODO: Port the remaining tests. test_frozen_lake, test_black_jack, test_cliff_world,
# test_cart_pole, test_acrobot and test_rest_api_server_wrapper need a running environments
# server and the removed apiserver.h API. test_mountain_car, test_taxi, test_tiled_cart_pole,
# test_vector_env_wrapper, test_vector_time_step and test_numpy_cpp_utils use the gymfcpp API
