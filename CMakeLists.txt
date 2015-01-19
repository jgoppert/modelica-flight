cmake_minimum_required(VERSION 2.8)
find_program(OMC omc)
find_program(MAKE make)

project(modelica-flight NONE)

enable_testing()

add_custom_target(build ALL
	DEPENDS QuadTest)

add_custom_command(OUTPUT QuadTest
	COMMAND ${OMC} -q -s ${CMAKE_SOURCE_DIR}/test/QuadTest.mo Modelica ${CMAKE_SOURCE_DIR}/Flight.mo
	COMMAND ${MAKE} -f QuadTest.makefile
	DEPENDS ${CMAKE_SOURCE_DIR}/test/QuadTest.mo ${CMAKE_SOURCE_DIR}/Flight.mo)

add_test(QuadTest QuadTest)
