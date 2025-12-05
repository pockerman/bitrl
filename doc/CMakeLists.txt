FIND_PACKAGE(Doxygen)

IF (NOT DOXYGEN_FOUND)
	return()
ENDIF()

# Configure Doxyfile
SET(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
SET(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

# just output XML since this is what Breathe consumes
SET(DOXYGEN_GENERATE_HTML YES)
SET(DOXYGEN_GENERATE_XML NO)

configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

add_custom_target(doc
		COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMENT "Generating API documentation with Doxygen..."
		VERBATIM
)
