CMAKE_MINIMUM_REQUIRED(VERSION 3.20 FATAL_ERROR)
MESSAGE(STATUS "Using CMake ${CMAKE_VERSION}")

SET(CUBERLLIB_VERSION_MAJOR 2)
SET(CUBERLLIB_VERSION_MINOR 1)
SET(CUBERLLIB_VERSION_PATCH 0)
SET(CUBERLLIB_VERSION "${CUBERLLIB_VERSION_MAJOR}.${CUBERLLIB_VERSION_MINOR}.${CUBERLLIB_VERSION_PATCH}")

# -----------------------------------------------------------------------------
# Options. Configure with -D<OPTION>=ON|OFF
# -----------------------------------------------------------------------------
OPTION(ENABLE_EXAMPLES "Build the examples" ON)
OPTION(ENABLE_TESTS "Build the unit tests" OFF)
OPTION(USE_CUDA "Build with CUDA. libtorch must have been built with CUDA too" OFF)
OPTION(CUBERL_LOG "Log various messages from the library" ON)
OPTION(CUBERL_WERROR "Treat compiler warnings as errors in Debug builds" OFF)

IF(USE_CUDA)
	PROJECT(cuberl VERSION ${CUBERLLIB_VERSION} LANGUAGES CXX C CUDA)
	MESSAGE(STATUS "Compiling with CUDA support")
ELSE()
	PROJECT(cuberl VERSION ${CUBERLLIB_VERSION} LANGUAGES CXX C)
ENDIF()

MESSAGE(STATUS "cuberl lib Version ${CUBERLLIB_VERSION}")

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
	SET(CUBERL_DEBUG ON)
	SET(CUBERL_PRINT_DBG_MSGS ON)
ENDIF()

IF(USE_CUDA)
	SET(CMAKE_CUDA_STANDARD 17)
ENDIF()

# -----------------------------------------------------------------------------
# Dependencies. Anything not in a system prefix is found through
# CMAKE_PREFIX_PATH, for example
#   -DCMAKE_PREFIX_PATH="<bitrl prefix>;<libtorch dir>;<Chrono prefix>"
# -----------------------------------------------------------------------------

# bitrl brings Boost, Eigen and, depending on how it was built, Chrono,
# OpenCV and MQTT. bitrl_WITH_* say which.
FIND_PACKAGE(bitrl 2 REQUIRED)
MESSAGE(STATUS "Found bitrl ${bitrl_VERSION} in ${bitrl_DIR}")
MESSAGE(STATUS "bitrl built with Chrono: ${bitrl_WITH_CHRONO}, OpenCV: ${bitrl_WITH_OPENCV}, MQTT: ${bitrl_WITH_MQTT}")

# libtorch; it ships TorchConfig.cmake in <libtorch>/share/cmake/Torch
FIND_PACKAGE(Torch REQUIRED CONFIG)
SET(USE_PYTORCH ON)
MESSAGE(STATUS "Found Torch ${Torch_VERSION} in ${Torch_DIR}")

# std::execution::par in libstdc++ needs TBB
FIND_PACKAGE(TBB REQUIRED)

IF(ENABLE_TESTS)
	FIND_PACKAGE(GTest REQUIRED)
ENDIF()

# Exposed to the code through cuberl_config.h
SET(USE_LOG ${CUBERL_LOG})
SET(USE_RL ON)
SET(USE_OPENCV ${bitrl_WITH_OPENCV})
SET(USE_MQTT ${bitrl_WITH_MQTT})
SET(BITRL_CHRONO ${bitrl_WITH_CHRONO})
SET(DATA_SET_FOLDER ${PROJECT_SOURCE_DIR}/data)

MESSAGE(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")

# The generated headers go in the build tree, not the source tree
SET(CUBERL_GENERATED_INCLUDE_DIR ${PROJECT_BINARY_DIR}/include)
CONFIGURE_FILE(config.h.in ${CUBERL_GENERATED_INCLUDE_DIR}/cuberl/base/cuberl_config.h @ONLY)
CONFIGURE_FILE(version.h.in ${CUBERL_GENERATED_INCLUDE_DIR}/cuberl/base/cuberl_version.h @ONLY)

# Warnings for cuberl's own targets
FUNCTION(cuberl_set_warnings target)
	TARGET_COMPILE_OPTIONS(${target} PRIVATE -Wall -Wpedantic)
	IF(CUBERL_WERROR AND CMAKE_BUILD_TYPE STREQUAL "Debug")
		TARGET_COMPILE_OPTIONS(${target} PRIVATE -Werror)
	ENDIF()
ENDFUNCTION()

# -----------------------------------------------------------------------------
# The library
# -----------------------------------------------------------------------------
FILE(GLOB SRCS src/cuberl/base/*.cpp
		src/cuberl/rl/*.cpp
		src/cuberl/rl/*/*.cpp
		src/cuberl/rl/algorithms/*.cpp
		src/cuberl/rl/algorithms/*/*.cpp
		src/cuberl/utils/*.cpp
		src/cuberl/maths/*.cpp
		src/cuberl/maths/statistics/*.cpp
		src/cuberl/maths/statistics/distributions/*.cpp
		src/cuberl/maths/optimization/*.cpp
	)

ADD_LIBRARY(cuberllib SHARED ${SRCS})
TARGET_INCLUDE_DIRECTORIES(cuberllib PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CUBERL_GENERATED_INCLUDE_DIR}>)
TARGET_INCLUDE_DIRECTORIES(cuberllib SYSTEM PUBLIC ${TORCH_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(cuberllib PUBLIC bitrl::bitrl ${TORCH_LIBRARIES} TBB::tbb)
cuberl_set_warnings(cuberllib)
INSTALL(TARGETS cuberllib LIBRARY DESTINATION lib)

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
