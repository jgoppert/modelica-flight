find_program(GCOV_PROG gcov)
find_library(GCOV_LIBRARIES NAMES gcov libgcov.a HINTS
	/usr/lib/gcc/x86_64-linux-gnu/4.8/)
include(FindPackageHandleStandardArgs)
message(${GCOV_LIBRARIES})
# handle the QUIETLY and REQUIRED arguments and set GCOV_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(GCOV DEFAULT_MSG GCOV_PROG GCOV_LIBRARIES)
