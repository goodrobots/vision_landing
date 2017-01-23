cmake_minimum_required (VERSION 2.6)
project (track_targets)
add_executable(track_targets track_targets.cpp)

set(cpp_compile_flags "-std=gnu++11")
add_definitions(${cpp_compile_flags})

find_package(OpenCV REQUIRED)

find_package(PkgConfig)
pkg_check_modules(pc_aruco REQUIRED aruco)
set(aruco_definitions ${pc_aruco_cflags_other})


find_path(aruco_include_dir aruco/aruco.h
    hints ${pc_aruco_includedir} ${pc_aruco_include_dirs}
    path_suffixes aruco )
    
find_library (aruco_library aruco
    hints ${pc_aruco_libdir} ${pc_aruco_library_dirs} )
    
include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ARUCO_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(aruco  DEFAULT_MSG
                                  aruco_library aruco_include_dir)

mark_as_advanced(aruco_include_dir aruco_library )

set(aruco_libraries ${aruco_library} )
set(aruco_include_dirs ${aruco_include_dir} )

include_directories(${aruco_include_dir} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(track_targets ${aruco_library} ${OpenCV_LIBS})
