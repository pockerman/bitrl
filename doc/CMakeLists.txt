FIND_PACKAGE(Doxygen)

IF (NOT DOXYGEN_FOUND)
	return()
ENDIF()


SET(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/docs)

# Configure Doxyfile
SET(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
SET(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

# just output XML since this is what Breathe consumes
SET(DOXYGEN_GENERATE_HTML YES)
SET(DOXYGEN_GENERATE_XML NO)

# don't generate documentation from implementation details
#SET(DOXYGEN_EXCLUDE_PATTERNS "*/extern/*")

# print paths relative to include directory
#SET(DOXYGEN_STRIP_FROM_PATH "${PROJECT_SOURCE_DIR}/src/bitrl")

configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)


add_custom_target(doc
		COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMENT "Generating API documentation with Doxygen..."
		VERBATIM
)

# Note: use doxygen_add_docs(doxygen-doc ALL ...) if you want your documentation
# to be created by default each time you build. Without the keyword you need to
# explicitly invoke building of the 'doc' target.
#doxygen_add_docs(doxygen-doc ${PROJECT_SOURCE_DIR}/src/bitrl COMMENT "Generating API documentation with Doxygen")
#
#
#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/conf.py.in
#               ${CMAKE_CURRENT_BINARY_DIR}/conf.py @ONLY)
#
#add_custom_target(
#  sphinx-doc ALL
#  COMMAND ${SPHINX_EXECUTABLE} -b html -c ${CMAKE_CURRENT_BINARY_DIR}
#          ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
#  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#  COMMENT "Generating API documentation with Sphinx"
#  VERBATIM)
#
#add_dependencies(sphinx-doc doxygen-doc)
