cmake_minimum_required (VERSION 3.2)
project (track_targets)
add_executable(track_targets track_targets.cpp)

set(cpp_compile_flags "-std=gnu++11")
add_definitions(${cpp_compile_flags})

find_package(OpenCV REQUIRED)

find_package(PkgConfig)
pkg_check_modules(PC_ARUCO QUIET REQUIRED aruco)
set(ARUCO_DEFINITIONS ${PC_ARUCO_CFLAGS_OTHER})

find_path(ARUCO_INCLUDE_DIR aruco/aruco.h
    HINTS ${PC_ARUCO_INCLUDEDIR} ${PC_ARUCO_INCLUDE_DIRS}
    PATH_SUFFIXES aruco )

find_library (ARUCO_LIBRARY NAMES aruco
    HINTS ${PC_ARUCO_LIBDIR} ${PC_ARUCO_LIBRARY_DIRS} )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Aruco DEFAULT_MSG
    ARUCO_LIBRARY ARUCO_INCLUDE_DIR)

mark_as_advanced( ARUCO_INCLUDE_DIR ARUCO_LIBRARY )

set( ARUCO_LIBRARIES ${ARUCO_LIBRARY} )
set( ARUCO_INCLUDE_DIRS ${ARUCO_INCLUDE_DIR} )

include_directories(${ARUCO_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(track_targets ${ARUCO_LIBRARY} ${OpenCV_LIBS})

install(PROGRAMS track_targets DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/..)
