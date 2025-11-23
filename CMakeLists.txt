CMAKE_MINIMUM_REQUIRED(VERSION 3.20)
MESSAGE(STATUS "Using CMake ${CMAKE_VERSION}")


SET(BITRL_VERSION_MAJOR 1)
SET(BITRL_VERSION_MINOR 17)
SET(BITRL_VERSION_PATCH 0)

SET(BITRL_VERSION "${BITRL_VERSION_MAJOR}.${BITRL_VERSION_MINOR}.${BITRL_VERSION_PATCH}")
MESSAGE(STATUS "bitrllib Version ${BITRL_VERSION}")

# create a CMake project
PROJECT(bitrl VERSION ${BITRL_VERSION} LANGUAGES CXX)

# -----------------------------------------------------------------------------
# Prevent in-source builds.
# -----------------------------------------------------------------------------

IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
	MESSAGE( FATAL_ERROR  "In-source build is not possible. Choose an empty directory for build output.")
ENDIF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

# Be sure to avoid troubles with library paths when using old policy
IF(COMMAND cmake_policy)
	CMAKE_POLICY(SET CMP0003 NEW)
	CMAKE_POLICY(SET CMP0048 NEW)
ENDIF(COMMAND cmake_policy)

OPTION(ENABLE_TESTS OFF)
#OPTION(ENABLE_EXAMPLES OFF)
OPTION(ENABLE_DOCS OFF)
OPTION(ENABLE_WEBOTS OFF)
#OPTION(ENABLE_CHRONO OFF)

SET(ENABLE_EXAMPLES ON)
SET(ENABLE_CHRONO ON)
SET(ENABLE_OPENCV ON)
SET(ENABLE_MQTT ON)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_STANDARD 20)
SET(CMAKE_CXX_STANDARD_REQUIRED True)
SET(CMAKE_LINKER_FLAGS "-pthread")

IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(CMAKE_INSTALL_PREFIX  ${PROJECT_SOURCE_DIR}/install/dbg)
	SET(CMAKE_CXX_FLAGS "-g -fPIC") # -Wall -Wextra")
    SET(BITRL_DEBUG ON)
	
ELSE(CMAKE_BUILD_TYPE STREQUAL "Release")
	SET(CMAKE_INSTALL_PREFIX  ${PROJECT_SOURCE_DIR}/install/opt)
	SET(CMAKE_CXX_FLAGS "-O2 -fPIC")
    SET(BITRL_DEBUG OFF)
ENDIF()

# Find packages that we need
FIND_PACKAGE(Boost 1.74.0 REQUIRED)
FIND_PACKAGE(BLAS REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(PahoMqttCpp REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

SET(BITRL_OPENCV ON)
SET(BITRL_MQTT ON)

IF(Boost_FOUND)
	MESSAGE( STATUS  "Found needed Boost C++ library.")
	SET(Boost_USE_SHARED_LIBS ON)
ELSE()
	MESSAGE( FATAL_ERROR  "Boost C++ library is required but not found.")
ENDIF()

# where to install the library
IF(ENABLE_TESTS_FLAG)
	# find gtest
	FIND_PACKAGE(GTest REQUIRED)

	IF(GTest_FOUND)
		INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})
		LINK_DIRECTORIES("/usr/local/lib")
		MESSAGE( STATUS  "Found Gtest at ${GTEST_INCLUDE_DIRS}.")
    ELSE()
	    MESSAGE( FATAL_ERROR  "ENABLE_TESTS_FLAG is ON but could not find GTest library.")
	ENDIF()
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

IF(ENABLE_WEBOTS)
	# we have webots include the directories
    SET(WEBOTS_LIB_DIRS ${PROJECT_SOURCE_DIR}/external/webots/lib/controller)    
    SET(WEBOTS_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/external/webots/include/controller/cpp)
	SET(BITRL_WEBOTS ON)
	
	INCLUDE_DIRECTORIES(${WEBOTS_INCLUDE_DIRS})
	LINK_DIRECTORIES(${WEBOTS_LIB_DIRS})
ENDIF()

IF(ENABLE_CHRONO)

	FIND_PACKAGE(Chrono  REQUIRED)
	IF (NOT Chrono_FOUND)
  		MESSAGE("Could not find Chrono or one of its required modules")
  		RETURN()
	ENDIF()

	SET(BITRL_CHRONO ON)
	SET(BITRL_CHRONO_TARGETS ${CHRONO_TARGETS})

	# Manually set this as for some reason it does not work
	# otherwise
    INCLUDE_DIRECTORIES(/usr/local/include/chrono_thirdparty/HACD)
	INCLUDE_DIRECTORIES(/usr/local/include/chrono/collision/bullet)
	LINK_DIRECTORIES(/usr/local/lib)
ENDIF()

configure_file(config.h.in ${PROJECT_SOURCE_DIR}/src/bitrl/bitrl_config.h @ONLY)
configure_file(version.h.in ${PROJECT_SOURCE_DIR}/src/bitrl/bitrl_version.h @ONLY)

MESSAGE(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
MESSAGE(STATUS "Boost lib: ${Boost_LIBRARY_DIR}")
MESSAGE(STATUS "Boost include: ${Boost_INCLUDE_DIRS}")
MESSAGE(STATUS "Install dst: ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "Eigen3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIR}")

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(/usr/local/include/) # mqtt
INCLUDE_DIRECTORIES(src/)

FILE(GLOB SRCS src/bitrl/*.cpp
		src/bitrl/network/*.cpp
		src/bitrl/sensors/*cpp
		src/bitrl/envs/*.cpp
		src/bitrl/envs/api_server/*.cpp
		src/bitrl/envs/gymnasium/*.cpp
		src/bitrl/envs/gymnasium/toy_text/*.cpp
		src/bitrl/envs/gymnasium/classic_control/*.cpp
		src/bitrl/envs/gymnasium/classic_control/vector/*.cpp
		src/bitrl/envs/gdrl/*.cpp
		src/bitrl/envs/multi_armed_bandits/*.cpp
		src/bitrl/envs/grid_world/*.cpp
		src/bitrl/envs/connect2/*.cpp
		src/bitrl/envs/webots_envs/*.cpp
		src/bitrl/envs/gymnasium/box2d/*.cpp
		src/bitrl/boards/arduino/*.cpp
		src/bitrl/rigid_bodies/*.cpp
		src/bitrl/rigid_bodies/webots_robots/*.cpp
		src/bitrl/dynamics/*.cpp
		src/bitrl/utils/*.cpp
		src/bitrl/utils/io/*.cpp
		src/bitrl/utils/io/tensor_board_server/*.cpp
		src/bitrl/utils/maths/statistics/distributions/*.cpp
		#src/bitrl/utils/geometry/*.cpp
		#src/bitrl/utils/geometry/shapes/*.cpp
		#src/bitrl/utils/geometry/mesh/*.cpp
		#src/bitrl/utils/trajectory/*.cpp
		src/bitrl/boards/arduino/*.cpp
		src/bitrl/systems/*.cpp
   )

ADD_LIBRARY(bitrllib SHARED ${SRCS})

SET_TARGET_PROPERTIES(bitrllib PROPERTIES LINKER_LANGUAGE CXX)
#target_link_libraries(bitrllib PUBLIC Eigen3::Eigen)

INSTALL(TARGETS bitrllib DESTINATION ${CMAKE_INSTALL_PREFIX})
MESSAGE(STATUS "Installation destination at: ${CMAKE_INSTALL_PREFIX}")

IF(ENABLE_EXAMPLES)
	# Add the examples
	ADD_SUBDIRECTORY(examples)
ENDIF()

IF(ENABLE_TESTS)
	# Add the tests
	ADD_SUBDIRECTORY(tests)
ENDIF()

IF(ENABLE_DOCS)
	# Add the tests
	ADD_SUBDIRECTORY(doc)
ENDIF()
