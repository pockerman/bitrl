CMAKE_MINIMUM_REQUIRED(VERSION 3.20 FATAL_ERROR)
MESSAGE(STATUS "Using CMake ${CMAKE_VERSION}")

# version.txt is the only place the version is kept. release-please bumps it
# when a release is made; do not edit it by hand
FILE(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" BITRLLIB_VERSION LIMIT_COUNT 1)
IF(NOT BITRLLIB_VERSION MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
	MESSAGE(FATAL_ERROR "version.txt must hold a version such as 1.2.3, not '${BITRLLIB_VERSION}'")
ENDIF()
SET(BITRLLIB_VERSION_MAJOR ${CMAKE_MATCH_1})
SET(BITRLLIB_VERSION_MINOR ${CMAKE_MATCH_2})
SET(BITRLLIB_VERSION_PATCH ${CMAKE_MATCH_3})

# -----------------------------------------------------------------------------
# Options. Configure with -D<OPTION>=ON|OFF
# -----------------------------------------------------------------------------
OPTION(ENABLE_EXAMPLES "Build the examples" ON)
OPTION(ENABLE_TESTS "Build the unit tests" OFF)
OPTION(USE_CUDA "Build with CUDA. libtorch must have been built with CUDA too" OFF)
OPTION(BITRL_LOG "Log various messages from the library" ON)
OPTION(BITRL_WERROR "Treat compiler warnings as errors in Debug builds" OFF)

IF(USE_CUDA)
	PROJECT(bitrl VERSION ${BITRLLIB_VERSION} LANGUAGES CXX C CUDA)
	MESSAGE(STATUS "Compiling with CUDA support")
ELSE()
	PROJECT(bitrl VERSION ${BITRLLIB_VERSION} LANGUAGES CXX C)
ENDIF()

MESSAGE(STATUS "bitrl lib Version ${BITRLLIB_VERSION}")

IF(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
	MESSAGE(FATAL_ERROR "In-source build is not possible. Choose an empty directory for build output.")
ENDIF()

# Release unless asked otherwise
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Debug or Release" FORCE)
ENDIF()

SET(CMAKE_CXX_STANDARD 20)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(BITRL_DEBUG ON)
	SET(BITRL_PRINT_DBG_MSGS ON)
ENDIF()

IF(USE_CUDA)
	SET(CMAKE_CUDA_STANDARD 17)
ENDIF()

# -----------------------------------------------------------------------------
# Dependencies. Anything not in a system prefix is found through
# CMAKE_PREFIX_PATH, for example
#   -DCMAKE_PREFIX_PATH="<bitsim prefix>;<libtorch dir>;<Chrono prefix>"
# -----------------------------------------------------------------------------

# bitsim brings Boost, Eigen and, depending on how it was built, Chrono,
# OpenCV and MQTT. bitsim_WITH_* say which.
FIND_PACKAGE(bitsim 4 REQUIRED)
MESSAGE(STATUS "Found bitsim ${bitsim_VERSION} in ${bitsim_DIR}")
MESSAGE(STATUS "bitsim built with Chrono: ${bitsim_WITH_CHRONO}, OpenCV: ${bitsim_WITH_OPENCV}, MQTT: ${bitsim_WITH_MQTT}")

# libtorch; it ships TorchConfig.cmake in <libtorch>/share/cmake/Torch
FIND_PACKAGE(Torch REQUIRED CONFIG)
SET(USE_PYTORCH ON)
MESSAGE(STATUS "Found Torch ${Torch_VERSION} in ${Torch_DIR}")

# std::execution::par in libstdc++ needs TBB
FIND_PACKAGE(TBB REQUIRED)

IF(ENABLE_TESTS)
	FIND_PACKAGE(GTest REQUIRED)
ENDIF()

# Exposed to the code through bitrl_config.h
SET(USE_LOG ${BITRL_LOG})
SET(USE_RL ON)
SET(USE_OPENCV ${bitsim_WITH_OPENCV})
SET(USE_MQTT ${bitsim_WITH_MQTT})
SET(BITSIM_CHRONO ${bitsim_WITH_CHRONO})
SET(DATA_SET_FOLDER ${PROJECT_SOURCE_DIR}/data)

MESSAGE(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")

# The generated headers go in the build tree, not the source tree
SET(BITRL_GENERATED_INCLUDE_DIR ${PROJECT_BINARY_DIR}/include)
CONFIGURE_FILE(config.h.in ${BITRL_GENERATED_INCLUDE_DIR}/bitrl/base/bitrl_config.h @ONLY)
CONFIGURE_FILE(version.h.in ${BITRL_GENERATED_INCLUDE_DIR}/bitrl/base/bitrl_version.h @ONLY)

# Warnings for bitrl's own targets
FUNCTION(bitrl_set_warnings target)
	TARGET_COMPILE_OPTIONS(${target} PRIVATE -Wall -Wpedantic)
	IF(BITRL_WERROR AND CMAKE_BUILD_TYPE STREQUAL "Debug")
		TARGET_COMPILE_OPTIONS(${target} PRIVATE -Werror)
	ENDIF()
ENDFUNCTION()

# -----------------------------------------------------------------------------
# The library
# -----------------------------------------------------------------------------
FILE(GLOB_RECURSE SRCS CONFIGURE_DEPENDS src/bitrl/*.cpp)

ADD_LIBRARY(bitrllib SHARED ${SRCS})
TARGET_INCLUDE_DIRECTORIES(bitrllib PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${BITRL_GENERATED_INCLUDE_DIR}>)
TARGET_INCLUDE_DIRECTORIES(bitrllib SYSTEM PUBLIC ${TORCH_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(bitrllib PUBLIC bitsim::bitsim ${TORCH_LIBRARIES} TBB::tbb)
bitrl_set_warnings(bitrllib)
INSTALL(TARGETS bitrllib LIBRARY DESTINATION lib)

IF(ENABLE_EXAMPLES)
	ADD_SUBDIRECTORY(examples)
ELSE()
	MESSAGE(STATUS "Examples have not been enabled")
ENDIF()

IF(ENABLE_TESTS)
	ENABLE_TESTING()
	ADD_SUBDIRECTORY(tests)
ELSE()
	MESSAGE(STATUS "Tests have not been enabled")
ENDIF()
