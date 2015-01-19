cmake_minimum_required(VERSION 2.8)

project(modelica-flight C)

option(COVERAGE "compute coverage" OFF)

#set(CMAKE_C_COMPILER "/usr/bin/clang")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
list(APPEND CMAKE_SHARED_LIBRARY_LINK_FLAGS -rdynamic)
find_package(OMC REQUIRED)
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(Threads REQUIRED)
find_library(MATH_LIB m)

if(COVERAGE)
    message(STATUS "coverage enabled")
    find_package(GCOV REQUIRED)
    link_libraries(${GCOV_LIBRARY})
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ftest-coverage -fprofile-arcs")
endif()

include(modelica)

enable_testing()

include_directories(/usr/include/omc/c)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_definitions(
    -DOPENMODELICA_XML_FROM_FILE_AT_RUNTIME
    )
link_directories(/usr/lib/omc /usr/lib/)
link_libraries(
    ${OMC_C_RUNTIME}
    ${LAPACK_LIBRARIES}
    ${BLAS_LIBRARIES}
    ${MATH_LIB}
    gc
    ${CMAKE_THREAD_LIBS_INIT}
    )

add_modelica_model(Quadrotor ${CMAKE_SOURCE_DIR}/test/Quadrotor.mo ${CMAKE_SOURCE_DIR}/Flight.mo Modelica)
add_test(Quadrotor Quadrotor)

add_modelica_model(DoublePendulum ${CMAKE_SOURCE_DIR}/test/DoublePendulum.mo "" Modelica)
add_test(DoublePendulum DoublePendulum)

set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
include(CPack)

install(TARGETS Quadrotor DESTINATION share/${PROJECT_NAME})
install(FILES ${Quadrotor_data} DESTINATION share/${PROJECT_NAME})
install(FILES Flight.mo DESTINATION share/${PROJECT_NAME})

# vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 ft=cmake :
