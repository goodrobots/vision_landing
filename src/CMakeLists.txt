cmake_minimum_required (VERSION 2.8)
project (track_targets)
find_package(aruco REQUIRED )
find_package(OpenCV REQUIRED)

option(USE_TIMERS "Set to OFF to disable timers" OFF)
iF(USE_TIMERS)
add_definitions(-DUSE_TIMERS)
ENDIF()

add_executable(track_targets track_targets.cpp)

set(cpp_compile_flags "-std=gnu++11")
add_definitions(${cpp_compile_flags})

include_directories(${aruco_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(track_targets aruco ${aruco_LIBS} ${OpenCV_LIBS})
link_directories(${aruco_LIB_DIR} ${OpenCV_INSTALL_PATH}/lib})
install(PROGRAMS track_targets DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/..)
