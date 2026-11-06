CMAKE_MINIMUM_REQUIRED(VERSION 3.20)
MESSAGE(STATUS "Using CMake ${CMAKE_VERSION}")

SET(BITRL_VERSION_MAJOR 2)
SET(BITRL_VERSION_MINOR 4)
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


# configure packages. These can be overridden from the command line
# e.g. cmake -DENABLE_CHRONO=OFF -DENABLE_TESTS=ON
OPTION(ENABLE_WEBOTS "Build with Webots support" OFF)
OPTION(ENABLE_EXAMPLES "Build the examples" ON)
OPTION(ENABLE_TESTS "Build the unit tests" OFF)
OPTION(ENABLE_OPENCV "Build with OpenCV support" ON)
OPTION(ENABLE_MQTT "Build with MQTT support" ON)
OPTION(ENABLE_CHRONO "Build with Project Chrono support" ON)
OPTION(BITRL_LOG "Log various messages from the library" ON)
OPTION(BITRL_IRRLICHT "Use Irrlicht for visualization. Requires ENABLE_CHRONO" ON)
OPTION(BITRL_WERROR "Treat compiler warnings as errors in Debug builds" OFF)

IF(NOT ENABLE_CHRONO)
	SET(BITRL_IRRLICHT OFF)
ENDIF()

# build options
IF(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type" FORCE)
ENDIF()

SET(CMAKE_CXX_STANDARD 20)
SET(CMAKE_CXX_STANDARD_REQUIRED True)
SET(CMAKE_LINKER_FLAGS "-pthread")
SET(ROBOTS_DATA_DIR ${PROJECT_SOURCE_DIR}/robots)
SET(SENSORS_DATA_DIR ${PROJECT_SOURCE_DIR}/sensors)
SET(WORLDS_DATA_DIR ${PROJECT_SOURCE_DIR}/worlds)

IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(CMAKE_INSTALL_PREFIX  ${PROJECT_SOURCE_DIR}/install/dbg)
	SET(CMAKE_CXX_FLAGS "-g -ggdb -fPIC -DBOOST_LOG_DYN_LINK -Wall -Wpedantic") # -Wall -Wextra")
	IF(BITRL_WERROR)
		STRING(APPEND CMAKE_CXX_FLAGS " -Werror")
	ENDIF()
    SET(BITRL_DEBUG ON)
	
ELSE(CMAKE_BUILD_TYPE STREQUAL "Release")
	SET(CMAKE_INSTALL_PREFIX  ${PROJECT_SOURCE_DIR}/install/opt)
	SET(CMAKE_CXX_FLAGS "-O2 -fPIC -DBOOST_LOG_DYN_LINK")
    SET(BITRL_DEBUG OFF)
ENDIF()

# Find packages that we need
FIND_PACKAGE(Boost 1.74.0 REQUIRED)
FIND_PACKAGE(BLAS REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
#FIND_PACKAGE(libirrlicht-dev REQUIRED)

IF(ENABLE_MQTT)
    FIND_PACKAGE(PahoMqttCpp REQUIRED)
    SET(BITRL_MQTT ON)
ENDIF()

IF(ENABLE_OPENCV)
    FIND_PACKAGE(OpenCV REQUIRED)
    SET(BITRL_OPENCV ON)
ENDIF()

IF(Boost_FOUND)
	MESSAGE( STATUS  "Found needed Boost C++ library.")
	SET(Boost_USE_SHARED_LIBS ON)
	SET(Boost_USE_STATIC_LIBS ON)
ELSE()
	MESSAGE( FATAL_ERROR  "Boost C++ library is required but not found.")
ENDIF()

IF(ENABLE_TESTS)
	FIND_PACKAGE(GTest REQUIRED)
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

	FIND_PACKAGE(Chrono  REQUIRED COMPONENTS Irrlicht)
	IF (NOT Chrono_FOUND)
  		MESSAGE("Could not find Chrono or one of its required modules")
  		RETURN()
	ENDIF()

	SET(BITRL_CHRONO ON)
	SET(BITRL_CHRONO_TARGETS ${CHRONO_TARGETS})

	# Manually set this as for some reason it does not work
	# otherwise
	INCLUDE_DIRECTORIES(/usr/include/irrlicht)
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
		src/bitrl/boards/arduino/*.cpp
		src/bitrl/control/*.cpp
		src/bitrl/dynamics/*.cpp
		src/bitrl/envs/*.cpp
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
		src/bitrl/network/*.cpp
		src/bitrl/planning/*.cpp
		src/bitrl/rigid_bodies/*.cpp
		src/bitrl/rigid_bodies/chrono_robots/*.cpp
		src/bitrl/rigid_bodies/chrono_robots/impl/*.cpp
		src/bitrl/rigid_bodies/chrono_robots/impl/turtle_bot/*.cpp
		src/bitrl/rigid_bodies/webots_robots/*.cpp
		src/bitrl/sensors/*.cpp
		src/bitrl/sensors/messages/*.cpp
		src/bitrl/sensors/backends/*.cpp
		src/bitrl/systems/*.cpp
		src/bitrl/utils/*.cpp
		src/bitrl/utils/io/*.cpp
		src/bitrl/utils/maths/statistics/distributions/*.cpp
		src/bitrl/utils/render/*.cpp
		src/bitrl/worlds/*.cpp
		src/bitrl/worlds/chrono_worlds/*.cpp

		#src/bitrl/utils/geometry/*.cpp
		#src/bitrl/utils/geometry/shapes/*.cpp
		#src/bitrl/utils/geometry/mesh/*.cpp
		#src/bitrl/utils/trajectory/*.cp

   )

ADD_LIBRARY(bitrllib SHARED ${SRCS})

SET_TARGET_PROPERTIES(bitrllib PROPERTIES LINKER_LANGUAGE CXX)

# Everything that links against bitrllib inherits these
TARGET_COMPILE_DEFINITIONS(bitrllib PUBLIC BOOST_LOG_DYN_LINK)
TARGET_LINK_LIBRARIES(bitrllib PUBLIC boost_log pthread)

IF(ENABLE_CHRONO)
	TARGET_LINK_LIBRARIES(bitrllib PUBLIC Chrono::Chrono_core)
	IF(BITRL_IRRLICHT)
		TARGET_LINK_LIBRARIES(bitrllib PUBLIC Chrono::Chrono_irrlicht)
	ENDIF()
ENDIF()

IF(BITRL_MQTT)
	TARGET_LINK_LIBRARIES(bitrllib PUBLIC paho-mqttpp3 paho-mqtt3as)
ENDIF()

IF(BITRL_OPENCV)
	TARGET_LINK_LIBRARIES(bitrllib PUBLIC ${OpenCV_LIBS})
ENDIF()

INSTALL(TARGETS bitrllib DESTINATION ${CMAKE_INSTALL_PREFIX})
MESSAGE(STATUS "Installation destination at: ${CMAKE_INSTALL_PREFIX}")

IF(ENABLE_EXAMPLES)
	# Add the examples
	ADD_SUBDIRECTORY(examples)
ENDIF()

IF(ENABLE_TESTS)
	# Add the tests
	ENABLE_TESTING()
	ADD_SUBDIRECTORY(tests)
ENDIF()
