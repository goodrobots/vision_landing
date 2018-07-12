cmake_minimum_required (VERSION 3.2)
project (track_targets)
add_executable(track_targets track_targets.cpp)

set(cpp_compile_flags "-std=gnu++11")
add_definitions(${cpp_compile_flags})

find_package(OpenCV REQUIRED)

find_package(aruco REQUIRED)

include_directories(${aruco_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(track_targets ${aruco_LIBS} ${OpenCV_LIBS})
link_directories(${aruco_LIB_DIR} ${OpenCV_INSTALL_PATH}/lib})
install(PROGRAMS track_targets DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/..)
