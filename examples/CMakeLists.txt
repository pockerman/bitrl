# bitrl_add_example(<name>
#                    [REQUIRES CHRONO IRRLICHT MQTT OPENCV ...]
#                    [FILES <file> ...]
#                    [SMOKE_TEST] [SMOKE_ARGS <arg> ...])
#
# Builds <name>.cpp, or <name>/<name>.cpp when <name> is a directory, into the
# executable <name>, if bitsim was built with everything in REQUIRES (see
# bitsim_WITH_*). FILES are copied next to the executable. With SMOKE_TEST and
# ENABLE_TESTS=ON, ctest runs the example with SMOKE_ARGS and expects it to
# exit with 0; ctest -L examples runs only these. Each example links only
# bitrllib; bitsim, libtorch and TBB come with it.
FUNCTION(bitrl_add_example name)
	CMAKE_PARSE_ARGUMENTS(EXAMPLE "SMOKE_TEST" "" "REQUIRES;FILES;SMOKE_ARGS" ${ARGN})
	FOREACH(feature ${EXAMPLE_REQUIRES})
		IF(NOT bitsim_WITH_${feature})
			MESSAGE(STATUS "Example ${name} needs bitsim with ${feature}; not building it")
			RETURN()
		ENDIF()
	ENDFOREACH()

	SET(dir ${CMAKE_CURRENT_SOURCE_DIR})
	SET(out ${CMAKE_CURRENT_BINARY_DIR})
	IF(IS_DIRECTORY ${dir}/${name})
		SET(dir ${dir}/${name})
		SET(out ${out}/${name})
	ENDIF()

	ADD_EXECUTABLE(${name} ${dir}/${name}.cpp)
	TARGET_INCLUDE_DIRECTORIES(${name} PRIVATE ${dir})
	TARGET_LINK_LIBRARIES(${name} PRIVATE bitrllib)
	IF(bitsim_WITH_IRRLICHT)
		TARGET_COMPILE_DEFINITIONS(${name} PRIVATE IRRLICHT_NO_XML)
	ENDIF()
	SET_TARGET_PROPERTIES(${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${out})

	FOREACH(file ${EXAMPLE_FILES})
		CONFIGURE_FILE(${dir}/${file} ${out}/${file} COPYONLY)
	ENDFOREACH()

	IF(EXAMPLE_SMOKE_TEST AND ENABLE_TESTS)
		ADD_TEST(NAME example_${name} COMMAND ${name} ${EXAMPLE_SMOKE_ARGS} WORKING_DIRECTORY ${out})
		SET_TESTS_PROPERTIES(example_${name} PROPERTIES LABELS examples TIMEOUT 600)
	ENDIF()
ENDFUNCTION()

# The environments, and wrapping a bitsim robot as one
ADD_SUBDIRECTORY(envs)
ADD_SUBDIRECTORY(robotics)

ADD_SUBDIRECTORY(example_1)
ADD_SUBDIRECTORY(example_2)
ADD_SUBDIRECTORY(example_3)
ADD_SUBDIRECTORY(example_4)
ADD_SUBDIRECTORY(example_6)
ADD_SUBDIRECTORY(example_7)
# we need CUDA for this to work
IF(USE_CUDA)
	ADD_SUBDIRECTORY(example_8)
ENDIF()
ADD_SUBDIRECTORY(example_9)
ADD_SUBDIRECTORY(example_10)
ADD_SUBDIRECTORY(example_11)
ADD_SUBDIRECTORY(example_12)
ADD_SUBDIRECTORY(example_13)
ADD_SUBDIRECTORY(example_14)
ADD_SUBDIRECTORY(example_15)
ADD_SUBDIRECTORY(example_16)
ADD_SUBDIRECTORY(example_17)
ADD_SUBDIRECTORY(example_18)
ADD_SUBDIRECTORY(example_19)
ADD_SUBDIRECTORY(example_21)
ADD_SUBDIRECTORY(example_22)
ADD_SUBDIRECTORY(example_23)
#ADD_SUBDIRECTORY(example_24)
ADD_SUBDIRECTORY(example_25)
