cmake_minimum_required(VERSION 2.8)

project(modelica-flight C)

find_program(OMC omc)
if (${OMC} STREQUAL "OMC-NOTFOUND")
    message(FATAL_ERROR "omc required: (Ubuntu) sudo apt-get install openmodelica")
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
include(modelica)

enable_testing()

include_directories(/usr/include/omc/c)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_definitions(-DOPENMODELICA_XML_FROM_FILE_AT_RUNTIME)
link_directories(/usr/lib/omc /usr/lib/)
link_libraries(SimulationRuntimeC lapack blas m gc pthread)

add_modelica(Quadrotor ${CMAKE_SOURCE_DIR}/test/Quadrotor.mo ${CMAKE_SOURCE_DIR}/Flight.mo Modelica)
add_test(Quadrotor Quadrotor)

add_modelica(DoublePendulum ${CMAKE_SOURCE_DIR}/test/DoublePendulum.mo "" Modelica)
add_test(DoublePendulum DoublePendulum)

set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
include(CPack)

install(TARGETS Quadrotor DESTINATION share/${PROJECT_NAME})
install(FILES ${Quadrotor_data} DESTINATION share/${PROJECT_NAME})
install(FILES Flight.mo DESTINATION share/${PROJECT_NAME})

# vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 ft=cmake :
