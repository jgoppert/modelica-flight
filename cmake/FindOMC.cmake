find_program(OMC_COMPILER omc)
find_library(OMC_C_RUNTIME SimulationRuntimeC HINTS /usr/lib/omc)
include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(OMC DEFAULT_MSG OMC_COMPILER OMC_C_RUNTIME)
